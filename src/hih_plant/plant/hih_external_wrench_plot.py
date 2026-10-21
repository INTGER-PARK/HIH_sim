#!/usr/bin/env python3
import array
import math
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rosidl_runtime_py.utilities import get_message
from rosidl_parser.definition import AbstractNestedType, Array as RosArray, BasicType, NamespacedType

from plant.cop_boundary_display import (
    COP_BOUNDARY_SUFFIXES, COP_SAFETY_DEBUG_SUFFIX,
    common_cop_boundary, cop_boundary_snapshot, cop_controller_status,
    cop_safety_from_debug, render_cop_boundary,
)

try:
    from PyQt5 import QtCore, QtWidgets
    QT_BINDING = "PyQt5"
except ImportError:
    try:
        from PySide6 import QtCore, QtWidgets
        QT_BINDING = "PySide6"
    except ImportError as exc:
        raise SystemExit(
            "Qt binding not found. Install `python3-pyqt5` or `PySide6`, and `pyqtgraph`."
        ) from exc

try:
    import pyqtgraph as pg
    from pyqtgraph.graphicsItems.LegendItem import ItemSample
except ImportError as exc:
    raise SystemExit("`pyqtgraph` is not installed. Install it before running this node.") from exc


COLORS = (
    "#d1495b",
    "#edae49",
    "#00798c",
    "#66a182",
    "#9c6644",
    "#8d99ae",
    "#ef476f",
    "#118ab2",
    "#ffd166",
    "#06d6a0",
)
COP_TYPE_NAME = "hih_interfaces/msg/CenterOfPressure"
# Plot-only unit conversion. ROS messages, controller math, rosbag and CSV stay
# in their native units; only displayed curve values are converted.
RAD_TO_DEG = 180.0 / math.pi
MINITRONE_STATE_TYPE = "hih_interfaces/msg/MinitroneState"
MOB_OBSERVER_INPUT_TYPE = "hih_interfaces/msg/MobObserverInput"
CONTACT_DIAGNOSTICS_TYPE = "hih_interfaces/msg/ContactDiagnostics"
INPUT_TYPE = "hih_interfaces/msg/Input"
ATTITUDE_CMD_TYPE = "hih_interfaces/msg/AttitudeCmd"
WRENCH_TYPE = "hih_interfaces/msg/Wrench"


def angular_plot_transform(
    topic_name: str, type_name: str, field_path: str
) -> Tuple[float, str]:
    """Return (display scale, unit) for known angular ROS fields.

    Native ROS units are intentionally unchanged.  The generic PyQt plot applies
    this transform only when a curve is created, so debugging is in degrees
    while controllers/recordings keep their original radian conventions.
    """

    # Strongly typed HIH messages.
    if type_name == MINITRONE_STATE_TYPE:
        if field_path.startswith("rpy["):
            return RAD_TO_DEG, "deg"
        if field_path.startswith("w_rpy["):
            return RAD_TO_DEG, "deg/s"
        if field_path.startswith("a_rpy["):
            return RAD_TO_DEG, "deg/s^2"
        if field_path.startswith("servo["):
            # Plant already publishes servo measurement in degrees.
            return 1.0, "deg"

    if type_name == MOB_OBSERVER_INPUT_TYPE:
        if field_path.startswith("rpy["):
            return RAD_TO_DEG, "deg"
        if field_path.startswith("w_rpy["):
            return RAD_TO_DEG, "deg/s"

    if type_name == CONTACT_DIAGNOSTICS_TYPE:
        if field_path.startswith("relative_rpy["):
            return RAD_TO_DEG, "deg"
        if field_path.startswith("relative_angular_velocity["):
            return RAD_TO_DEG, "deg/s"

    if type_name == INPUT_TYPE and field_path in {
        "u[4]", "u[5]", "u[6]", "u[7]"
    }:
        # Allocator publishes servo tilt commands in radians at u[4:8].
        # u[0:4] are motor-speed commands and intentionally remain native.
        return RAD_TO_DEG, "deg"

    if type_name == ATTITUDE_CMD_TYPE and field_path in {
        "roll_ref", "pitch_ref", "yaw_ref"
    }:
        # AttitudeCmd.msg is already degrees.
        return 1.0, "deg"

    if type_name == WRENCH_TYPE:
        if field_path.startswith("force["):
            return 1.0, "N"
        if field_path.startswith("moment["):
            return 1.0, "N*m"

    if topic_name.endswith("/admittance/debug_z"):
        # data[13]=roll, data[14]=pitch, both raw state radians.
        if field_path in {"data[13]", "data[14]"}:
            return RAD_TO_DEG, "deg"

    if topic_name.endswith("/cop_safety/debug"):
        # target/output pitch and yaw are already generated in degrees.
        if field_path in {"data[5]", "data[6]", "data[17]", "data[18]"}:
            return 1.0, "deg"

    if topic_name.endswith((
        "/cop_safety_pitch_offset_deg",
        "/cop_safety_yaw_offset_deg",
    )):
        return 1.0, "deg"

    return 1.0, ""


def curve_display_name(
    panel_index: int, topic_name: str, field_path: str, unit: str
) -> str:
    unit_suffix = f" [{unit}]" if unit else ""
    return f"[P{panel_index + 1}] {topic_name} :: {field_path}{unit_suffix}"


def automatic_time_bounds(
    earliest_time: float, latest_time: float, window_sec: float
) -> Tuple[float, float]:
    """Return a rolling time range without reserving empty history on the left.

    All curve timestamps are seconds from time.perf_counter().  Before a curve
    has accumulated a full window, show only the interval for which the active
    curves actually contain samples.  Once enough history exists, keep the
    normal rolling window [latest-window, latest].
    """
    if not (math.isfinite(earliest_time) and math.isfinite(latest_time)):
        return 0.0, 1.0
    if latest_time < earliest_time:
        earliest_time, latest_time = latest_time, earliest_time

    window = max(float(window_sec), 1e-3)
    xmin = max(earliest_time, latest_time - window)
    if latest_time - xmin > 1e-9:
        return xmin, latest_time

    # One-sample startup: keep a small readable interval rather than a
    # zero-width ViewBox range.  This disappears as soon as more data arrives.
    half_span = min(0.5, 0.05 * window)
    half_span = max(0.05, half_span)
    return max(0.0, latest_time - half_span), latest_time + half_span


class ToggleLegendSample(ItemSample):
    """Legend sample that toggles one curve or a related group of plot items."""

    def __init__(self, item, linked_items=None) -> None:
        super().__init__(item)
        self.linked_items = list(linked_items or [item])
        self.label = None

    def toggle_visibility(self) -> None:
        visible = any(item.isVisible() for item in self.linked_items)
        new_visible = not visible
        for item in self.linked_items:
            item.setVisible(new_visible)
        opacity = 1.0 if new_visible else 0.35
        self.setOpacity(opacity)
        if self.label is not None:
            self.label.setOpacity(opacity)
        self.update()

    def mouseClickEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.toggle_visibility()
        event.accept()


class ToggleLegendLabel(pg.LabelItem):
    """Clickable legend text linked to the same visibility toggle as its icon."""

    def __init__(self, text: str, sample: ToggleLegendSample, **kwargs) -> None:
        super().__init__(text, **kwargs)
        self.sample = sample

    def mouseClickEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.sample.toggle_visibility()
        event.accept()


class ClickableLegendItem(pg.LegendItem):
    """Legend whose icon and text both toggle their associated visualization."""

    def addItem(self, item, name, linked_items=None) -> None:
        sample = ToggleLegendSample(item, linked_items)
        label = ToggleLegendLabel(
            name,
            sample,
            color=self.opts["labelTextColor"],
            justify="left",
            size=self.opts["labelTextSize"],
        )
        sample.label = label
        self.items.append((sample, label))
        self._addItemToLayout(sample, label)
        self.updateSize()


@dataclass
class TopicInfo:
    name: str
    type_name: str
    message_class: type
    metadata_fields: List[str]
    observed_fields: Set[str]
    latest_flattened: Dict[str, float]
    message_count: int = 0
    first_message_logged: bool = False
    latest_rx_time: Optional[float] = None

    def all_fields(self) -> List[str]:
        return sorted(set(self.metadata_fields) | self.observed_fields)


@dataclass
class CurveRecord:
    key: str
    topic: str
    field_path: str
    display_name: str
    color: str
    panel_index: int
    plot_item: pg.PlotDataItem
    times: Deque[float]
    values: Deque[float]
    value_scale: float = 1.0
    unit: str = ""
    dirty: bool = True


@dataclass
class CopRecord:
    valid: bool
    y: float
    z: float
    normal_force: float
    trail_y: Deque[float]
    trail_z: Deque[float]


@dataclass
class CopBoundaryItems:
    z_upper: pg.PlotDataItem
    z_lower: pg.PlotDataItem
    y_positive: pg.PlotDataItem
    y_negative: pg.PlotDataItem


@dataclass
class CopPanel:
    container: object
    plot_widget: pg.PlotWidget
    status_label: object
    hat_point_item: pg.ScatterPlotItem
    hat_trail_item: pg.PlotDataItem
    real_point_item: pg.ScatterPlotItem
    real_trail_item: pg.PlotDataItem
    solver_point_item: pg.ScatterPlotItem
    solver_trail_item: pg.PlotDataItem
    boundary_items: CopBoundaryItems
    base: str = ""


def is_numeric_value(value: object) -> bool:
    if isinstance(value, bool):
        return False
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


def is_numeric_sequence(value: object) -> bool:
    if isinstance(value, (str, bytes)):
        return False
    if isinstance(value, dict):
        return False
    try:
        items = list(value) if not isinstance(value, array.array) else list(value)
    except TypeError:
        return False
    if len(items) == 0:
        return False
    return all(is_numeric_value(item) for item in items)


def flatten_numeric_message(msg: object, prefix: str = "") -> Dict[str, float]:
    data: Dict[str, float] = {}
    if is_numeric_value(msg):
        data[prefix] = float(msg)
        return data

    if is_numeric_sequence(msg):
        for index, item in enumerate(msg):
            data[f"{prefix}[{index}]"] = float(item)
        return data

    if hasattr(msg, "__slots__"):
        for slot in msg.__slots__:
            field_name = slot.lstrip("_")
            child_prefix = field_name if not prefix else f"{prefix}.{field_name}"
            child_value = getattr(msg, slot)
            data.update(flatten_numeric_message(child_value, child_prefix))
    return data


def sanitize_numeric_map(values: Dict[str, float]) -> Dict[str, float]:
    sanitized: Dict[str, float] = {}
    for key, value in values.items():
        if math.isfinite(value):
            sanitized[key] = value
    return sanitized


NUMERIC_PRIMITIVE_TYPES = {
    "float",
    "double",
    "long double",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
}


def namespaced_type_to_string(slot_type: NamespacedType) -> str:
    return "/".join([*slot_type.namespaces, slot_type.name])


def discover_numeric_fields(message_class: type, prefix: str = "") -> List[str]:
    fields: List[str] = []
    slot_types = getattr(message_class, "SLOT_TYPES", [])
    slots = getattr(message_class, "__slots__", [])

    for slot_name, slot_type in zip(slots, slot_types):
        field_name = slot_name.lstrip("_")
        field_prefix = field_name if not prefix else f"{prefix}.{field_name}"

        if isinstance(slot_type, BasicType):
            if slot_type.typename in NUMERIC_PRIMITIVE_TYPES:
                fields.append(field_prefix)
            continue

        if isinstance(slot_type, AbstractNestedType):
            value_type = slot_type.value_type
            if isinstance(value_type, BasicType) and value_type.typename in NUMERIC_PRIMITIVE_TYPES:
                if isinstance(slot_type, RosArray):
                    for index in range(slot_type.size):
                        fields.append(f"{field_prefix}[{index}]")
                else:
                    fields.append(f"{field_prefix}[]")
            elif isinstance(value_type, NamespacedType) and isinstance(slot_type, RosArray):
                try:
                    nested_class = get_message(namespaced_type_to_string(value_type))
                except Exception:
                    continue
                nested_fields = discover_numeric_fields(nested_class)
                for index in range(slot_type.size):
                    for nested_field in nested_fields:
                        fields.append(f"{field_prefix}[{index}].{nested_field}")
            else:
                fields.append(f"{field_prefix}[]")
            continue

        if isinstance(slot_type, NamespacedType):
            try:
                nested_class = get_message(namespaced_type_to_string(slot_type))
            except Exception:
                continue
            fields.extend(discover_numeric_fields(nested_class, field_prefix))

    return sorted(fields)


class TopicPlotNode(Node):
    def __init__(self, *, initial_topic: str = "") -> None:
        super().__init__("hih_topic_plot")
        self.window_sec = float(self.declare_parameter("window_sec", 10.0).value)
        self.refresh_hz = float(self.declare_parameter("refresh_hz", 10.0).value)
        self.discovery_period_sec = float(self.declare_parameter("discovery_period_sec", 2.0).value)
        self.auto_add_limit = max(1, int(self.declare_parameter("auto_add_limit", 8).value))
        self.plot_rows = max(1, min(4, int(self.declare_parameter("plot_rows", 2).value)))
        self.plot_columns = max(1, min(4, int(self.declare_parameter("plot_columns", 2).value)))
        self.plate_size_y = abs(float(self.declare_parameter("plate_size_y", 0.40).value))
        self.plate_size_z = abs(float(self.declare_parameter("plate_size_z", 0.38).value))
        self.cop_trail_length = max(
            1, int(self.declare_parameter("cop_trail_length", 100).value)
        )
        default_topic = initial_topic or ""
        self.default_topic = str(self.declare_parameter("topic", default_topic).value)
        self.history_limit = max(200, int(self.window_sec * max(self.refresh_hz, 1.0) * 8.0))
        self.start_time: Optional[float] = None
        self.lock = threading.RLock()
        self.executor: Optional[SingleThreadedExecutor] = None

        self.topic_infos: Dict[str, TopicInfo] = {}
        self.subscriptions_by_topic: Dict[str, object] = {}
        self.curves_by_key: Dict[str, CurveRecord] = {}
        self.cop_records: Dict[str, CopRecord] = {}

    def discover_topics(self) -> Dict[str, TopicInfo]:
        discovered: Dict[str, TopicInfo] = {}
        previous_infos = self.topic_infos
        for topic_name, type_names in self.get_topic_names_and_types():
            if not type_names:
                continue
            type_name = type_names[0]
            try:
                message_class = get_message(type_name)
            except Exception:
                continue

            fields = discover_numeric_fields(message_class)
            if not fields:
                continue
            observed_fields = set()
            if topic_name in previous_infos:
                observed_fields = set(previous_infos[topic_name].observed_fields)
            discovered[topic_name] = TopicInfo(
                name=topic_name,
                type_name=type_name,
                message_class=message_class,
                metadata_fields=fields,
                observed_fields=observed_fields,
                latest_flattened=dict(previous_infos[topic_name].latest_flattened) if topic_name in previous_infos else {},
                message_count=previous_infos[topic_name].message_count if topic_name in previous_infos else 0,
                first_message_logged=previous_infos[topic_name].first_message_logged if topic_name in previous_infos else False,
                latest_rx_time=previous_infos[topic_name].latest_rx_time if topic_name in previous_infos else None,
            )
        with self.lock:
            self.topic_infos = dict(sorted(discovered.items()))
            return dict(self.topic_infos)

    def ensure_subscription(self, topic_name: str) -> None:
        with self.lock:
            if topic_name in self.subscriptions_by_topic:
                return
            topic_info = self.topic_infos.get(topic_name)
            if topic_info is None:
                return
            self.subscriptions_by_topic[topic_name] = self.create_subscription(
                topic_info.message_class,
                topic_name,
                self._make_topic_callback(topic_name),
                10,
            )
            self.get_logger().info(f"subscribed to {topic_name} [{topic_info.type_name}]")
            if self.executor is not None:
                self.executor.wake()

    def _make_topic_callback(self, topic_name: str) -> Callable[[object], None]:
        def callback(msg: object) -> None:
            try:
                now = time.perf_counter()
                with self.lock:
                    if self.start_time is None:
                        self.start_time = now
                    t = now - self.start_time
                flattened = sanitize_numeric_map(flatten_numeric_message(msg))
                with self.lock:
                    topic_info = self.topic_infos.get(topic_name)
                    if topic_info is not None:
                        topic_info.message_count += 1
                        topic_info.observed_fields.update(flattened.keys())
                        topic_info.latest_flattened = dict(flattened)
                        topic_info.latest_rx_time = now
                        if not topic_info.first_message_logged:
                            preview_keys = sorted(flattened.keys())[:6]
                            self.get_logger().info(
                                f"first message on {topic_name}: {len(flattened)} numeric fields, sample keys={preview_keys}"
                            )
                            topic_info.first_message_logged = True
                        if topic_info.type_name == COP_TYPE_NAME:
                            record = self.cop_records.get(topic_name)
                            if record is None:
                                record = CopRecord(
                                    valid=False,
                                    y=0.0,
                                    z=0.0,
                                    normal_force=0.0,
                                    trail_y=deque(maxlen=self.cop_trail_length),
                                    trail_z=deque(maxlen=self.cop_trail_length),
                                )
                                self.cop_records[topic_name] = record
                            record.valid = bool(getattr(msg, "valid", False))
                            record.normal_force = float(
                                getattr(msg, "normal_force", 0.0)
                            )
                            if record.valid:
                                record.y = float(getattr(msg, "y", 0.0))
                                record.z = float(getattr(msg, "z", 0.0))
                                record.trail_y.append(record.y)
                                record.trail_z.append(record.z)
                    for curve in self.curves_by_key.values():
                        if curve.topic != topic_name:
                            continue
                        value = flattened.get(curve.field_path)
                        if value is None:
                            continue
                        curve.times.append(t)
                        curve.values.append(value * curve.value_scale)
                        curve.dirty = True
            except Exception as exc:
                self.get_logger().error(f"plot callback failed for {topic_name}: {exc}")

        return callback


class TopicPlotWindow(QtWidgets.QWidget):
    def __init__(self, node: TopicPlotNode, spawn_window: Optional[Callable[[], None]] = None) -> None:
        super().__init__()
        self.node = node
        self.spawn_window = spawn_window
        self.is_closing = False
        self.color_index = 0
        self.last_discovery_time = 0.0
        self.initial_auto_add_done = False
        self.current_topic_names: List[str] = []
        self.plot_rows = self.node.plot_rows
        self.plot_columns = self.node.plot_columns
        self.manual_x_range = False
        self.manual_y_ranges = [False] * 16
        self.active_cop_bases: List[str] = []

        self.setWindowTitle("ROS 2 Topic Plot")
        self.resize(1500, 900)
        self.setMinimumSize(480, 320)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        pg.setConfigOptions(antialias=False, foreground="#d8dee9", background="#11151c")

        root_layout = QtWidgets.QHBoxLayout(self)
        root_layout.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        self.main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.main_splitter.setChildrenCollapsible(False)
        root_layout.addWidget(self.main_splitter)

        controls_widget = QtWidgets.QWidget()
        controls_layout = QtWidgets.QVBoxLayout(controls_widget)
        self.main_splitter.addWidget(controls_widget)

        self.topic_filter_edit = QtWidgets.QLineEdit()
        self.topic_filter_edit.setPlaceholderText("filter topic")
        controls_layout.addWidget(self.topic_filter_edit)

        self.topic_list = QtWidgets.QListWidget()
        controls_layout.addWidget(self.topic_list, 2)

        self.topic_type_label = QtWidgets.QLabel("type: -")
        controls_layout.addWidget(self.topic_type_label)

        self.field_list = QtWidgets.QListWidget()
        self.field_list.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        controls_layout.addWidget(self.field_list, 2)

        self.add_button = QtWidgets.QPushButton("Add Selected Fields")
        controls_layout.addWidget(self.add_button)

        self.layout_combo = QtWidgets.QComboBox()
        for rows in range(1, 5):
            for columns in range(1, 5):
                self.layout_combo.addItem(f"Layout {rows} x {columns}", (rows, columns))
        initial_layout_index = (self.plot_rows - 1) * 4 + (self.plot_columns - 1)
        self.layout_combo.setCurrentIndex(initial_layout_index)
        controls_layout.addWidget(self.layout_combo)

        self.panel_combo = QtWidgets.QComboBox()
        controls_layout.addWidget(self.panel_combo)

        self.active_curve_list = QtWidgets.QListWidget()
        self.active_curve_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        controls_layout.addWidget(self.active_curve_list, 2)

        self.remove_button = QtWidgets.QPushButton("Remove Selected")
        self.clear_button = QtWidgets.QPushButton("Clear All")
        controls_layout.addWidget(self.remove_button)
        controls_layout.addWidget(self.clear_button)

        self.window_spin = QtWidgets.QDoubleSpinBox()
        self.window_spin.setRange(1.0, 120.0)
        self.window_spin.setValue(self.node.window_sec)
        self.window_spin.setSuffix(" s")
        controls_layout.addWidget(self.window_spin)

        self.resume_auto_range_button = QtWidgets.QPushButton("Resume Auto Range")
        controls_layout.addWidget(self.resume_auto_range_button)

        self.refresh_topics_button = QtWidgets.QPushButton("Refresh Topics")
        controls_layout.addWidget(self.refresh_topics_button)
        if self.spawn_window is not None:
            self.new_window_button = QtWidgets.QPushButton("Open New Window")
            controls_layout.addWidget(self.new_window_button)
        controls_layout.addStretch(1)

        plot_container = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(plot_container)
        self.main_splitter.addWidget(plot_container)
        self.main_splitter.setStretchFactor(0, 0)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setSizes([320, 1180])

        self.status_label = QtWidgets.QLabel(f"ready with {QT_BINDING} + pyqtgraph")
        plot_layout.addWidget(self.status_label)

        self.plot_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.plot_splitter.setChildrenCollapsible(False)
        plot_layout.addWidget(self.plot_splitter, 1)

        self.cop_container = QtWidgets.QWidget()
        cop_layout = QtWidgets.QVBoxLayout(self.cop_container)
        cop_layout.setContentsMargins(0, 0, 0, 0)
        self.plot_splitter.addWidget(self.cop_container)
        self.cop_panel_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.cop_panel_splitter.setChildrenCollapsible(False)
        cop_layout.addWidget(self.cop_panel_splitter)
        self.cop_panels = [self._create_cop_panel(), self._create_cop_panel()]
        self.cop_panel_splitter.setSizes([500, 500])

        self.time_plot_container = QtWidgets.QWidget()
        self.time_plot_container.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        self.plot_grid = QtWidgets.QGridLayout(self.time_plot_container)
        self.plot_grid.setContentsMargins(0, 0, 0, 0)
        self.plot_grid.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        self.plot_splitter.addWidget(self.time_plot_container)
        self.plot_splitter.setStretchFactor(0, 1)
        self.plot_splitter.setStretchFactor(1, 2)
        self.cop_container.hide()
        self.plot_widgets: List[pg.PlotWidget] = []
        for panel_index in range(16):
            plot_widget = pg.PlotWidget(title=f"Plot {panel_index + 1}")
            plot_widget.setMinimumSize(0, 0)
            plot_widget.setSizePolicy(
                QtWidgets.QSizePolicy.Ignored,
                QtWidgets.QSizePolicy.Ignored,
            )
            plot_widget.showGrid(x=True, y=True, alpha=0.25)

            # Plot timestamps are already elapsed seconds.  Do not let
            # pyqtgraph replace seconds with an SI-prefixed display such as ms.
            # Keeping raw-second ticks also makes screenshots directly
            # comparable with MuJoCo/CSV time values.
            plot_widget.setLabel("bottom", "time [s]")
            plot_widget.getAxis("bottom").enableAutoSIPrefix(False)

            plot_widget.setMouseEnabled(x=True, y=True)
            legend = ClickableLegendItem(offset=(10, 10))
            legend.setParentItem(plot_widget.plotItem.vb)
            plot_widget.plotItem.legend = legend
            plot_widget.setClipToView(True)
            plot_widget.setDownsampling(mode="peak")
            if self.plot_widgets:
                plot_widget.setXLink(self.plot_widgets[0])
            plot_widget.plotItem.vb.sigRangeChangedManually.connect(
                lambda axes, index=panel_index: self._on_plot_range_changed_manually(
                    index, axes
                )
            )
            self.plot_widgets.append(plot_widget)
        self._apply_plot_layout()

        self.topic_filter_edit.textChanged.connect(self._rebuild_topic_list)
        self.topic_list.currentRowChanged.connect(self._on_topic_selection_changed)
        self.add_button.clicked.connect(self._add_selected_fields)
        self.layout_combo.currentIndexChanged.connect(self._on_layout_changed)
        self.field_list.itemDoubleClicked.connect(self._on_field_double_clicked)
        self.remove_button.clicked.connect(self._remove_selected_curves)
        self.clear_button.clicked.connect(self._clear_curves)
        self.resume_auto_range_button.clicked.connect(self._resume_auto_range)
        self.refresh_topics_button.clicked.connect(self._refresh_topics)
        if self.spawn_window is not None:
            self.new_window_button.clicked.connect(self.spawn_window)

        interval_ms = max(10, int(1000.0 / max(self.node.refresh_hz, 1.0)))
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(interval_ms)

        self._refresh_topics()

    def _create_cop_panel(self) -> CopPanel:
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        self.cop_panel_splitter.addWidget(container)

        plot_widget = pg.PlotWidget(title="Center of Pressure")
        plot_widget.setMinimumSize(100, 100)
        plot_widget.showGrid(x=True, y=True, alpha=0.25)
        plot_widget.setLabel("bottom", "y", units="m")
        plot_widget.setLabel("left", "z", units="m")

        # CoP plate boundary has a fixed physical size.
        # Do not let changing plot items/data alter the view range.
        plot_widget.plotItem.enableAutoRange(x=False, y=False)

        # Keep equal metric scale in y and z.
        plot_widget.setAspectLocked(True)

        plot_widget.setMouseEnabled(x=True, y=True)
        
        legend = ClickableLegendItem(offset=(10, 10))
        legend.setParentItem(plot_widget.plotItem.vb)
        plot_widget.plotItem.legend = legend

        half_y = 0.5 * self.node.plate_size_y
        half_z = 0.5 * self.node.plate_size_z
        plot_widget.plot(
            [-half_y, half_y, half_y, -half_y, -half_y],
            [-half_z, -half_z, half_z, half_z, -half_z],
            pen=pg.mkPen(width=2),
            name="contact surface",
        )
        center_item = pg.ScatterPlotItem(
            [0.0], [0.0], symbol="+", size=14, pen=pg.mkPen(width=2)
        )
        plot_widget.addItem(center_item)
        legend.addItem(center_item, "plate center")

        hat_trail_item = plot_widget.plot(
            [], [], pen=pg.mkPen(color="#ffd200", width=1)
        )
        hat_point_item = pg.ScatterPlotItem(
            [], [], symbol="o", size=14,
            pen=pg.mkPen(color="#ffd200", width=2),
            brush=pg.mkBrush(255, 210, 0, 210),
        )
        plot_widget.addItem(hat_point_item)
        legend.addItem(
            hat_point_item,
            "estimated (MoB)",
            linked_items=[hat_point_item, hat_trail_item],
        )

        real_trail_item = plot_widget.plot(
            [], [], pen=pg.mkPen(color="#00d9ff", width=1)
        )
        real_point_item = pg.ScatterPlotItem(
            [], [], symbol="o", size=14,
            pen=pg.mkPen(color="#00d9ff", width=2),
            brush=pg.mkBrush(0, 217, 255, 210),
        )
        plot_widget.addItem(real_point_item)
        legend.addItem(
            real_point_item,
            "4-touch (cop_real)",
            linked_items=[real_point_item, real_trail_item],
        )

        solver_trail_item = plot_widget.plot(
            [], [], pen=pg.mkPen(color="#22c55e", width=1)
        )
        solver_point_item = pg.ScatterPlotItem(
            [], [], symbol="o", size=14,
            pen=pg.mkPen(color="#22c55e", width=2),
            brush=pg.mkBrush(34, 197, 94, 210),
        )
        plot_widget.addItem(solver_point_item)
        legend.addItem(
            solver_point_item,
            "plate support GT (cop_solver_real)",
            linked_items=[solver_point_item, solver_trail_item],
        )

        # One shared dashed safety boundary for attitude and translation.
        # The physical contact-surface outline remains independent.
        z_pen = pg.mkPen(color="#fb7185", width=2, style=QtCore.Qt.DashLine)
        y_pen = pg.mkPen(color="#38bdf8", width=2, style=QtCore.Qt.DashLine)
        z_upper = plot_widget.plot([], [], pen=z_pen)
        z_lower = plot_widget.plot([], [], pen=z_pen)
        y_positive = plot_widget.plot([], [], pen=y_pen)
        y_negative = plot_widget.plot([], [], pen=y_pen)
        legend.addItem(z_upper, "z safety boundary", linked_items=[z_upper, z_lower])
        legend.addItem(y_positive, "y safety boundary", linked_items=[y_positive, y_negative])
        boundary_items = CopBoundaryItems(z_upper, z_lower, y_positive, y_negative)

        status_label = QtWidgets.QLabel("waiting for CoP data")
        status_label.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Fixed,
        )
        status_label.setMinimumWidth(0)
        
        layout.addWidget(plot_widget, 1)
        layout.addWidget(status_label)
        panel = CopPanel(
            container=container,
            plot_widget=plot_widget,
            status_label=status_label,
            hat_point_item=hat_point_item,
            hat_trail_item=hat_trail_item,
            real_point_item=real_point_item,
            real_trail_item=real_trail_item,
            solver_point_item=solver_point_item,
            solver_trail_item=solver_trail_item,
            boundary_items=boundary_items,
        )
        self._reset_cop_panel_range(panel)
        return panel

    def _reset_cop_panel_range(self, panel: CopPanel) -> None:
        half_y = 0.5 * self.node.plate_size_y
        half_z = 0.5 * self.node.plate_size_z
        y_margin = max(0.02, 0.10 * self.node.plate_size_y)
        z_margin = max(0.02, 0.10 * self.node.plate_size_z)
        panel.plot_widget.setXRange(
            -half_y - y_margin, half_y + y_margin, padding=0.0
        )
        panel.plot_widget.setYRange(
            -half_z - z_margin, half_z + z_margin, padding=0.0
        )

    @property
    def active_panel_count(self) -> int:
        return self.plot_rows * self.plot_columns

    def _apply_plot_layout(self) -> None:
        # QGridLayout otherwise follows each PlotWidget's sizeHint.  In a
        # 2x4 layout, long legends/axis labels can therefore make one half of
        # the grid much wider than the other.  Reset every possible row/column
        # and give the active grid equal stretch factors.
        for column in range(4):
            self.plot_grid.setColumnStretch(column, 0)
            self.plot_grid.setColumnMinimumWidth(column, 0)
        for row in range(4):
            self.plot_grid.setRowStretch(row, 0)
            self.plot_grid.setRowMinimumHeight(row, 0)

        for column in range(self.plot_columns):
            self.plot_grid.setColumnStretch(column, 1)
        for row in range(self.plot_rows):
            self.plot_grid.setRowStretch(row, 1)

        for panel_index, plot_widget in enumerate(self.plot_widgets):
            self.plot_grid.removeWidget(plot_widget)
            if panel_index < self.active_panel_count:
                self.plot_grid.addWidget(
                    plot_widget,
                    panel_index // self.plot_columns,
                    panel_index % self.plot_columns,
                )
                plot_widget.show()
            else:
                plot_widget.hide()

        self.plot_grid.invalidate()
        self.plot_grid.activate()
        self.time_plot_container.updateGeometry()
        self.updateGeometry()

        previous_panel = self.panel_combo.currentIndex()
        self.panel_combo.clear()
        self.panel_combo.addItems(
            [f"Plot {index + 1}" for index in range(self.active_panel_count)]
        )
        self.panel_combo.setCurrentIndex(
            max(0, min(previous_panel, self.active_panel_count - 1))
        )

    def _on_layout_changed(self, _: int) -> None:
        layout = self.layout_combo.currentData()
        if layout is None:
            return
        self.plot_rows, self.plot_columns = layout

        with self.node.lock:
            curves_to_move = [
                curve for curve in self.node.curves_by_key.values()
                if curve.panel_index >= self.active_panel_count
            ]
        for curve in curves_to_move:
            old_widget = self.plot_widgets[curve.panel_index]
            old_widget.removeItem(curve.plot_item)
            curve.panel_index %= self.active_panel_count
            curve.display_name = curve_display_name(
                curve.panel_index, curve.topic, curve.field_path, curve.unit
            )
            new_widget = self.plot_widgets[curve.panel_index]
            new_widget.addItem(curve.plot_item)
            if new_widget.plotItem.legend is not None:
                new_widget.plotItem.legend.addItem(curve.plot_item, curve.display_name)

        self._apply_plot_layout()
        self._rebuild_active_curve_list()
        self._set_status_text(
            f"layout changed to {self.plot_rows} x {self.plot_columns}"
        )

    def _rebuild_active_curve_list(self) -> None:
        selected_names = {
            item.text() for item in self.active_curve_list.selectedItems()
        }
        self.active_curve_list.clear()
        with self.node.lock:
            display_names = [
                curve.display_name for curve in self.node.curves_by_key.values()
            ]
        for display_name in display_names:
            item = QtWidgets.QListWidgetItem(display_name)
            self.active_curve_list.addItem(item)
            if display_name in selected_names:
                item.setSelected(True)

    def _refresh_topics(self) -> None:
        if self.is_closing or not rclpy.ok():
            return
        previous_field_selection = self._selected_field_names()
        self.node.discover_topics()
        with self.node.lock:
            self.current_topic_names = list(self.node.topic_infos.keys())
        self._rebuild_topic_list()
        self._restore_field_selection(previous_field_selection)
        self._auto_add_default_topic_if_needed()

    def _rebuild_topic_list(self) -> None:
        selected_topic = self._selected_topic_name()
        pattern = self.topic_filter_edit.text().strip().lower()
        desired_topics = [
            topic_name
            for topic_name in self.current_topic_names
            if not pattern or pattern in topic_name.lower()
        ]
        visible_topics = [
            self.topic_list.item(index).text()
            for index in range(self.topic_list.count())
        ]
        # Topic discovery runs periodically. Avoid clearing an unchanged list,
        # because QListWidget.clear() resets the user's scrollbar to the top.
        if visible_topics == desired_topics:
            return

        scroll_bar = self.topic_list.verticalScrollBar()
        previous_scroll_value = scroll_bar.value()
        self.topic_list.blockSignals(True)
        self.topic_list.clear()
        for topic_name in desired_topics:
            self.topic_list.addItem(topic_name)
        self.topic_list.blockSignals(False)

        if selected_topic and selected_topic in desired_topics:
            self._select_topic_by_name(selected_topic)
        else:
            self.topic_list.clearSelection()
            self.topic_list.setCurrentRow(-1)
            self.field_list.clear()
            self.topic_type_label.setText("type: -")
            self._hide_cop_view()
        scroll_bar.setValue(
            min(previous_scroll_value, scroll_bar.maximum())
        )

    def _selected_topic_name(self) -> str:
        item = self.topic_list.currentItem()
        return "" if item is None else item.text()

    def _select_topic_by_name(self, topic_name: str) -> None:
        matches = self.topic_list.findItems(topic_name, QtCore.Qt.MatchExactly)
        if matches:
            self.topic_list.setCurrentItem(matches[0])
            self._on_topic_selection_changed(self.topic_list.currentRow())

    def _on_topic_selection_changed(self, _: int) -> None:
        topic_name = self._selected_topic_name()
        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
        previous_field_selection = self._selected_field_names()
        self.field_list.clear()
        if topic_info is None:
            self.topic_type_label.setText("type: -")
            self._hide_cop_view()
            return

        self.node.ensure_subscription(topic_name)
        self._show_cop_view_if_selected(topic_name, topic_info)
        self.topic_type_label.setText(f"type: {topic_info.type_name}")
        for field_name in topic_info.all_fields():
            item = QtWidgets.QListWidgetItem(field_name)
            self.field_list.addItem(item)
            if field_name in previous_field_selection:
                item.setSelected(True)

    def _hide_cop_view(self) -> None:
        self.active_cop_bases = []
        self.cop_container.hide()

    def _show_cop_view_if_selected(
        self, topic_name: str, topic_info: TopicInfo
    ) -> None:
        if topic_info.type_name != COP_TYPE_NAME:
            self._hide_cop_view()
            return

        cop_suffixes = ("/cop_hat", "/cop_real", "/cop_solver_real")
        suffix = next((value for value in cop_suffixes if topic_name.endswith(value)), None)
        if suffix is None:
            self._hide_cop_view()
            return
        cop_base = topic_name[:-len(suffix)]

        with self.node.lock:
            available_bases = {
                name.rsplit("/", 1)[0]
                for name, info in self.node.topic_infos.items()
                if info.type_name == COP_TYPE_NAME
                and name.endswith(cop_suffixes)
            }

        base_parts = cop_base.strip("/").split("/")
        role_index = next(
            (index for index, part in enumerate(base_parts)
             if part in ("leader", "follower")),
            None,
        )
        active_bases: List[str] = []
        if role_index is not None:
            for role in ("follower", "leader"):
                candidate_parts = list(base_parts)
                candidate_parts[role_index] = role
                candidate = "/" + "/".join(candidate_parts)
                if candidate in available_bases:
                    active_bases.append(candidate)
        if not active_bases:
            active_bases = [cop_base]

        was_hidden = self.cop_container.isHidden()
        self.active_cop_bases = active_bases[:len(self.cop_panels)]
        for panel_index, panel in enumerate(self.cop_panels):
            if panel_index >= len(self.active_cop_bases):
                panel.base = ""
                panel.container.hide()
                continue
            panel.base = self.active_cop_bases[panel_index]
            role = next(
                (part.capitalize() for part in panel.base.strip("/").split("/")
                 if part in ("leader", "follower")),
                panel.base,
            )
            panel.plot_widget.setTitle(f"{role} Center of Pressure")
            # Both drones face the payload along local +X; +Y appears on the left.
            panel.plot_widget.getViewBox().invertX(True)
            panel.container.show()
            for suffix in ("cop_hat", "cop_real", "cop_solver_real"):
                self.node.ensure_subscription(f"{panel.base}/{suffix}")
            for suffix in (*COP_BOUNDARY_SUFFIXES.values(), COP_SAFETY_DEBUG_SUFFIX):
                self.node.ensure_subscription(f"{panel.base}{suffix}")
        self.cop_container.show()
        if was_hidden:
            self.plot_splitter.setSizes([300, 520])
            self.cop_panel_splitter.setSizes([500, 500])
        self._update_cop_plot()

    def _auto_add_default_topic_if_needed(self) -> None:
        if self.initial_auto_add_done:
            return
        topic_name = self.node.default_topic
        if not topic_name:
            return
        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
        if topic_info is None:
            return

        self._select_topic_by_name(topic_info.name)
        added_count = 0
        for field_name in topic_info.all_fields():
            if added_count >= self.node.auto_add_limit:
                break
            if field_name.endswith("[]"):
                continue
            panel_index = added_count % self.active_panel_count
            if self._add_curve(topic_info.name, field_name, panel_index):
                added_count += 1
        if added_count > 0:
            suffix = ""
            if len(topic_info.all_fields()) > added_count:
                suffix = f" (limited to {added_count})"
            self._set_status_text(f"auto-added {added_count} field(s) from {topic_info.name}{suffix}")
            self.initial_auto_add_done = True

    def _add_selected_fields(self) -> None:
        topic_name = self._selected_topic_name()
        if not topic_name:
            self.status_label.setText("select a topic first")
            return
        selected_items = self.field_list.selectedItems()
        if not selected_items and self.field_list.currentItem() is not None:
            selected_items = [self.field_list.currentItem()]
        if not selected_items:
            self.status_label.setText("select one or more fields first")
            return

        added_count = 0
        for item in selected_items:
            if item.text().endswith("[]"):
                self.status_label.setText("wait for topic data so dynamic array indices appear")
                continue
            if self._add_curve(topic_name, item.text()):
                added_count += 1
        if added_count > 0:
            self.status_label.setText(f"added {added_count} field(s)")

    def _add_curve(
        self,
        topic_name: str,
        field_path: str,
        panel_index: Optional[int] = None,
    ) -> bool:
        key = f"{topic_name}:{field_path}"
        if key in self.node.curves_by_key:
            self.status_label.setText("field already added")
            return False

        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
        if topic_info is None:
            self.status_label.setText("topic info is not available")
            return False

        self.node.ensure_subscription(topic_name)
        color = COLORS[self.color_index % len(COLORS)]
        self.color_index += 1
        if panel_index is None:
            panel_index = self.panel_combo.currentIndex()
        panel_index = max(0, min(panel_index, self.active_panel_count - 1))
        value_scale, unit = angular_plot_transform(
            topic_name, topic_info.type_name, field_path
        )
        display_name = curve_display_name(
            panel_index, topic_name, field_path, unit
        )
        plot_item = self.plot_widgets[panel_index].plot(
            pen=pg.mkPen(color=color, width=2),
            name=display_name,
            autoDownsample=True,
            downsampleMethod="peak",
            clipToView=True,
        )
        record = CurveRecord(
            key=key,
            topic=topic_name,
            field_path=field_path,
            display_name=display_name,
            color=color,
            panel_index=panel_index,
            plot_item=plot_item,
            times=deque(maxlen=self.node.history_limit),
            values=deque(maxlen=self.node.history_limit),
            value_scale=value_scale,
            unit=unit,
        )
        with self.node.lock:
            self.node.curves_by_key[key] = record
        self.active_curve_list.addItem(display_name)
        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
            if topic_info is not None and field_path in topic_info.latest_flattened:
                if self.node.start_time is None:
                    self.node.start_time = time.perf_counter()
                seeded_time = 0.0 if self.node.start_time is None else max(0.0, time.perf_counter() - self.node.start_time)
                record.times.append(seeded_time)
                record.values.append(
                    topic_info.latest_flattened[field_path] * record.value_scale
                )
                record.dirty = True
        return True

    def _on_field_double_clicked(self, item: QtWidgets.QListWidgetItem) -> None:
        topic_name = self._selected_topic_name()
        if not topic_name:
            self.status_label.setText("select a topic first")
            return
        if item.text().endswith("[]"):
            self.status_label.setText("wait for topic data so dynamic array indices appear")
            return
        if self._add_curve(topic_name, item.text()):
            self.status_label.setText(f"added {item.text()}")

    def _remove_selected_curves(self) -> None:
        for item in list(self.active_curve_list.selectedItems()):
            self._remove_curve_by_name(item.text())

    def _remove_curve_by_name(self, display_name: str) -> None:
        key_to_remove = None
        with self.node.lock:
            curve_items = list(self.node.curves_by_key.items())
        for key, curve in curve_items:
            if curve.display_name == display_name:
                key_to_remove = key
                break
        if key_to_remove is None:
            return

        with self.node.lock:
            curve = self.node.curves_by_key.pop(key_to_remove)
        self.plot_widgets[curve.panel_index].removeItem(curve.plot_item)
        matches = self.active_curve_list.findItems(display_name, QtCore.Qt.MatchExactly)
        for item in matches:
            row = self.active_curve_list.row(item)
            self.active_curve_list.takeItem(row)

    def _clear_curves(self) -> None:
        with self.node.lock:
            display_names = [curve.display_name for curve in self.node.curves_by_key.values()]
        for display_name in display_names:
            self._remove_curve_by_name(display_name)

    def _on_plot_range_changed_manually(self, panel_index: int, axes: object) -> None:
        try:
            x_changed = bool(axes[0])  # type: ignore[index]
            y_changed = bool(axes[1])  # type: ignore[index]
        except (IndexError, TypeError):
            x_changed = True
            y_changed = True
        if x_changed:
            self.manual_x_range = True
        if y_changed:
            self.manual_y_ranges[panel_index] = True
        self._set_status_text("manual view; click Resume Auto Range to follow data")

    def _resume_auto_range(self) -> None:
        self.manual_x_range = False
        self.manual_y_ranges = [False] * len(self.plot_widgets)
        for panel in self.cop_panels:
            self._reset_cop_panel_range(panel)
        self._set_status_text("automatic range resumed")

    def _update_plot(self) -> None:
        self._sync_selected_topic_fields()
        with self.node.lock:
            curve_records = list(self.node.curves_by_key.values())
        if not curve_records:
            self._set_status_text("no active curves")
            return

        latest_time = 0.0
        earliest_visible_time = math.inf
        latest_visible_time = -math.inf
        earliest_any_time = math.inf
        latest_any_time = -math.inf
        visible_values_by_panel: List[List[float]] = [
            [] for _ in self.plot_widgets
        ]
        latest_summaries: List[str] = []
        curves_with_data = 0
        curve_sample_counts: List[str] = []
        for curve in curve_records:
            with self.node.lock:
                dirty = curve.dirty
                times = list(curve.times)
                values = list(curve.values)
                curve.dirty = False
            if times and values:
                if dirty:
                    curve.plot_item.setData(times, values)
                latest_time = max(latest_time, times[-1])
                earliest_any_time = min(earliest_any_time, times[0])
                latest_any_time = max(latest_any_time, times[-1])
                if curve.plot_item.isVisible():
                    earliest_visible_time = min(earliest_visible_time, times[0])
                    latest_visible_time = max(latest_visible_time, times[-1])
                    visible_values_by_panel[curve.panel_index].append(min(values))
                    visible_values_by_panel[curve.panel_index].append(max(values))
                    unit_suffix = f" {curve.unit}" if curve.unit else ""
                    latest_summaries.append(
                        f"{curve.field_path}={values[-1]:+.3f}{unit_suffix}"
                    )
                curve_sample_counts.append(f"{curve.field_path}:{len(values)}")
                curves_with_data += 1

        if curves_with_data == 0:
            first_curve = curve_records[0]
            with self.node.lock:
                topic_info = self.node.topic_infos.get(first_curve.topic)
            if topic_info is not None and topic_info.message_count > 0:
                self._set_status_text(
                    f"messages seen on {first_curve.topic} ({topic_info.message_count}), but `{first_curve.field_path}` has no numeric samples yet"
                )
            else:
                self._set_status_text(
                    f"waiting for messages on {first_curve.topic} | active curves={len(self.node.curves_by_key)}"
                )
            return

        # Use the visible curves' actual first/last samples for auto range.
        # Previously the x-axis always reserved a full window_sec behind the
        # global elapsed clock.  If the user selected a topic late, all samples
        # therefore appeared compressed into the right side of the plot.
        if math.isfinite(earliest_visible_time) and math.isfinite(latest_visible_time):
            range_start = earliest_visible_time
            range_end = latest_visible_time
        else:
            range_start = earliest_any_time
            range_end = latest_any_time
        xmin, xmax = automatic_time_bounds(
            range_start, range_end, self.window_spin.value()
        )
        if not self.manual_x_range:
            self.plot_widgets[0].setXRange(xmin, xmax, padding=0.01)
        for panel_index, visible_values in enumerate(visible_values_by_panel):
            if not visible_values or self.manual_y_ranges[panel_index]:
                continue
            ymin = min(visible_values)
            ymax = max(visible_values)
            if ymin == ymax:
                pad = 1.0 if abs(ymin) < 1e-9 else abs(ymin) * 0.1
                ymin -= pad
                ymax += pad
            else:
                pad = 0.1 * (ymax - ymin)
                ymin -= pad
                ymax += pad
            self.plot_widgets[panel_index].setYRange(ymin, ymax, padding=0.0)
        summary = " | ".join(latest_summaries[:4])
        if len(latest_summaries) > 4:
            summary += f" | +{len(latest_summaries) - 4} more"
        sample_summary = ", ".join(curve_sample_counts[:3])
        if len(curve_sample_counts) > 3:
            sample_summary += f", +{len(curve_sample_counts) - 3} more"
        self._set_status_text(
            (summary or "waiting for data...") + (f" | samples {sample_summary}" if sample_summary else "")
        )

    def _update_cop_plot(self) -> None:
        if not self.active_cop_bases:
            return

        def snapshot(topic_name: str):
            record = self.node.cop_records.get(topic_name)
            if record is None:
                return None
            return (
                record.valid,
                record.y,
                record.z,
                record.normal_force,
                list(record.trail_y),
                list(record.trail_z),
            )

        half_y = 0.5 * self.node.plate_size_y
        half_z = 0.5 * self.node.plate_size_z

        def render(record, point_item, trail_item, label: str, available: bool) -> str:
            if record is None:
                point_item.setData([], [])
                trail_item.setData([], [])
                return f"{label}: {'waiting' if available else 'off'}"
            valid, y_value, z_value, normal_force, trail_y, trail_z = record
            trail_item.setData(trail_y, trail_z)
            if not valid:
                point_item.setData([], [])
                return f"{label}: invalid, Fn={normal_force:.3f} N"
            point_item.setData([y_value], [z_value])
            region = (
                "inside" if abs(y_value) <= half_y and abs(z_value) <= half_z
                else "outside"
            )
            return (
                f"{label}: y={y_value:+.4f}, z={z_value:+.4f} m, "
                f"Fn={normal_force:.3f} N ({region})"
            )

        for panel in self.cop_panels:
            if not panel.base:
                continue

            # Subscribe when an optional controller starts after this view opens.
            for suffix in (*COP_BOUNDARY_SUFFIXES.values(), COP_SAFETY_DEBUG_SUFFIX):
                self.node.ensure_subscription(f"{panel.base}{suffix}")

            with self.node.lock:
                hat = snapshot(f"{panel.base}/cop_hat")
                real = snapshot(f"{panel.base}/cop_real")
                solver = snapshot(f"{panel.base}/cop_solver_real")
                hat_available = f"{panel.base}/cop_hat" in self.node.topic_infos
                real_available = f"{panel.base}/cop_real" in self.node.topic_infos
                solver_available = f"{panel.base}/cop_solver_real" in self.node.topic_infos
                now = time.perf_counter()
                boundary_states = {}
                for kind, suffix in COP_BOUNDARY_SUFFIXES.items():
                    info = self.node.topic_infos.get(f"{panel.base}{suffix}")
                    legacy = False
                    if info is None and kind == "attitude":
                        # Compatibility with older attitude safety publishers only.
                        info = self.node.topic_infos.get(f"{panel.base}{COP_SAFETY_DEBUG_SUFFIX}")
                        legacy = True
                    boundary_states[kind] = cop_boundary_snapshot(info, now, legacy)
            if not hat_available:
                hat = None
            if not real_available:
                real = None
            if not solver_available:
                solver = None
            hat_status = render(
                hat, panel.hat_point_item, panel.hat_trail_item, "estimated", hat_available
            )
            real_status = render(
                real, panel.real_point_item, panel.real_trail_item, "4-touch", real_available
            )
            solver_status = render(
                solver, panel.solver_point_item, panel.solver_trail_item,
                "plate support GT", solver_available
            )

            shared_state, missing_status = common_cop_boundary(boundary_states)
            boundary_status = render_cop_boundary(
                panel.boundary_items, shared_state, half_y, half_z, "safety", missing_status)
            controller_status = [cop_controller_status(kind, snapshot)
                                 for kind, snapshot in boundary_states.items()]
            panel.status_label.setText(" | ".join(
                [hat_status, real_status, solver_status, boundary_status, *controller_status]))

    def _sync_selected_topic_fields(self) -> None:
        topic_name = self._selected_topic_name()
        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
        if topic_info is None:
            return

        visible_fields = [self.field_list.item(index).text() for index in range(self.field_list.count())]
        expected_fields = topic_info.all_fields()
        if visible_fields == expected_fields:
            return

        selected_fields = {item.text() for item in self.field_list.selectedItems()}
        self.field_list.blockSignals(True)
        self.field_list.clear()
        for field_name in expected_fields:
            item = QtWidgets.QListWidgetItem(field_name)
            self.field_list.addItem(item)
            if field_name in selected_fields:
                item.setSelected(True)
        self.field_list.blockSignals(False)

    def _selected_field_names(self) -> Set[str]:
        return {item.text() for item in self.field_list.selectedItems()}

    def _restore_field_selection(self, field_names: Set[str]) -> None:
        if not field_names:
            return
        for index in range(self.field_list.count()):
            item = self.field_list.item(index)
            if item.text() in field_names:
                item.setSelected(True)

    def _set_status_text(self, text: str) -> None:
        if self.status_label.text() != text:
            self.status_label.setText(text)

    def _on_timer(self) -> None:
        if self.is_closing or not rclpy.ok():
            self.timer.stop()
            return
        try:
            now = time.perf_counter()
            if now - self.last_discovery_time >= self.node.discovery_period_sec:
                self.last_discovery_time = now
                self._refresh_topics()
            self._update_cop_plot()
            self._update_plot()
        except Exception as exc:
            self._set_status_text(f"plot update error: {exc}")
            if rclpy.ok():
                self.node.get_logger().error(f"plot update error: {exc}")

    def closeEvent(self, event) -> None:  # type: ignore[override]
        if self.is_closing:
            event.accept()
            return
        self.is_closing = True
        self.timer.stop()
        if self.node is not None:
            self.node.destroy_node()
        super().closeEvent(event)


def run_app(initial_topic: str = "") -> None:
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    executor = SingleThreadedExecutor()
    stop_event = threading.Event()
    windows: List[TopicPlotWindow] = []

    def create_window(topic_name: Optional[str] = None) -> TopicPlotWindow:
        node = TopicPlotNode(initial_topic=topic_name or initial_topic)
        node.executor = executor
        executor.add_node(node)
        window = TopicPlotWindow(node, spawn_window=create_window)
        windows.append(window)
        window.destroyed.connect(lambda *_ , w=window: windows.remove(w) if w in windows else None)
        window.show()
        return window

    def spin_executor() -> None:
        while not stop_event.is_set() and rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                break

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()
    create_window()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    exit_code = app.exec()
    stop_event.set()
    executor.shutdown()
    if spin_thread.is_alive():
        spin_thread.join(timeout=1.0)
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


def main() -> None:
    run_app()


def external_wrench_main() -> None:
    run_app()


if __name__ == "__main__":
    main()

