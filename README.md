# HIH_sim

두 UAV가 화물을 사이에 두고 접촉력으로 운반하는 **MuJoCo + ROS 2 협동 운송 시뮬레이터**입니다.
Leader와 Follower는 각자의 제어기와 관측기를 사용하며, 두 기체의 상호작용은 공통 MuJoCo world의 물리 접촉을 통해 발생합니다.

이 저장소는 협동 시뮬레이션 실행에 필요한 기능을 중심으로 구성합니다. 문서는 이 README 하나에서 관리합니다.

## 1. 실행 환경과 빌드

- Ubuntu 22.04 / ROS 2 Humble
- Python 3.10, C++17, MuJoCo 3.3.7
- NumPy, Eigen3, PyQt5, pyqtgraph
- ROS 2 빌드 도구, rosdep, rosbag2 Python 인터페이스

ROS 2와 rosdep이 설치·초기화된 환경에서 실행합니다.

```bash
git clone https://github.com/INTGER-PARK/HIH_sim.git
cd HIH_sim
source /opt/ros/humble/setup.bash
python3 -m pip install mujoco==3.3.7
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

이후 명령은 `HIH_sim` 루트에서 실행하며, 새 터미널마다 ROS 2와 `install/setup.bash`를 source합니다.

## 2. 패키지 구성

| 패키지 | 역할 |
|---|---|
| `hih_plant` | MuJoCo world, 기체·화물 모델, high-fidelity 모델, 외란, 관측 데이터 및 실시간 plot |
| `hih_controller` | Leader/Follower 제어기, allocator, MoB, admittance, CoP 보정, torque DoB |
| `hih_cmd` | 협동 launch, 키보드 teleop, Leader trajectory, 기체 외력 입력 |
| `hih_interfaces` | 제어·관측에 사용하는 ROS 메시지 |
| `hih_logging` | rosbag 기록과 CSV 변환 |

모델은 `src/hih_plant/xml/Cooperation.xml`에서 구성합니다. 기체 형상은 `Leader.xml`, `Follower.xml`, 화물·받침대는 `contact_box.xml`에 있습니다.

## 3. 기본 시뮬레이션 실행

```bash
ros2 launch hih_cmd cooperation_launch.py
```

기본 launch가 실행하는 노드입니다.

| 실행 항목 | 역할 |
|---|---|
| `cooperation_plant` | 두 기체와 화물을 포함한 공통 MuJoCo world 및 관측 데이터 발행 |
| `leader_wrench_controller` / `follower_wrench_controller` | 각 기체의 위치·자세 reference를 wrench 명령으로 변환 |
| `hih_allocator_controller` × 2 | 각 기체 wrench를 모터·서보 입력으로 할당 |
| `leader_second_order_momentum_observer` / `follower_second_order_momentum_observer` | 각 기체의 외력·외부 모멘트와 CoP 추정 |
| `follower_admittance_controller` | Follower admittance 처리. 프로세스는 기본 실행되지만 admittance는 OFF로 시작 |
| `hih_record_csv` | 시뮬레이션 데이터 기록. 기본 ON |

초기 reference는 기체의 초기 위치를 유지합니다. launch 자체가 접근·리프트·경로 추종 시퀀스를 시작하지는 않습니다.

| launch 인자 | 기본값 | 의미 |
|---|---|---|
| `viewer` | `true` | MuJoCo viewer 실행 |
| `record_data` | `true` | rosbag/CSV 기록 |
| `log_output_dir` | `src/hih_logging/recordings` | 기록 저장 위치 |
| `embedded_admittance` | `true` | Follower admittance 프로세스 포함 |
| `cop_touch` | `true` | 4-touch CoP 출력 |
| `cop_solver` | `true` | solver wrench 기반 CoP 출력 |
| `disturbance_config` | 패키지의 `config/disturbances.yaml` | 외란·바람 설정 파일 |

예를 들어 viewer와 데이터 기록을 끄려면 다음과 같이 실행합니다.

```bash
ros2 launch hih_cmd cooperation_launch.py viewer:=false record_data:=false
```

## 4. 명령 입력과 선택 제어기

각 키보드 노드는 별도 터미널에서 실행합니다. 실행된 터미널에 입력 포커스를 두고 조작합니다.

### Teleop

```bash
ros2 run hih_cmd hih_teleop
```

| 대상 | 수평 이동 | 높이 | yaw |
|---|---|---|---|
| Follower | `a/d`: −x/+x, `w/s`: −y/+y | `r/f`: +z/−z | `q/e` |
| Leader | `j/l`: +x/−x, `i/k`: −y/+y | `y/h`: +z/−z | `u/o` |

수평 이동은 각 기체의 yaw를 반영한 Body 방향이며 높이는 World z입니다. Leader만 조작하려면 `ros2 run hih_cmd hih_teleop_leader`를 사용합니다.

### Leader trajectory

```bash
ros2 run hih_cmd hih_trajectory
```

- `a`: 접근 → preload → lift 시퀀스 시작. Follower admittance도 활성화합니다.
- `l`: XY Lissajous, `t`: X 왕복, `r`: YZ 사각 경로를 현재 측정 위치에서 시작합니다.
- 같은 경로 키를 다시 누르면 해당 경로를 처음 시작한 측정 위치로 천천히 복귀합니다.

Trajectory가 제어권을 가진 동안 teleop의 Leader 명령은 양보합니다. 경로 종료·복귀가 Follower admittance를 자동으로 끄지는 않습니다.

### 선택 제어 노드

아래 노드는 기본 launch에 포함되지 않습니다. 각 명령을 필요할 때 별도로 실행합니다.

| 실행 명령 | 역할 |
|---|---|
| `ros2 run hih_controller leader_admittance_controller` | Leader admittance |
| `ros2 run hih_controller leader_normal_force` | Leader normal-force 제어 |
| `ros2 run hih_controller leader_cop_attitude_controller` | Leader 자세 offset을 통한 CoP 보정 |
| `ros2 run hih_controller follower_cop_attitude_controller` | Follower 자세 offset을 통한 CoP 보정 |
| `ros2 run hih_controller leader_cop_translation_controller` | Leader 위치 offset을 통한 CoP 보정 |
| `ros2 run hih_controller follower_cop_translation_controller` | Follower 위치 offset을 통한 CoP 보정 |
| `ros2 run hih_controller hih_torque_dob --ros-args -r __ns:=/leader` | Leader torque DoB |
| `ros2 run hih_controller hih_torque_dob --ros-args -r __ns:=/follower` | Follower torque DoB |

CoP 제어기는 `mob`와 `solver` 피드백을 선택할 수 있습니다. 공통 safety boundary 기본값은 `src/hih_controller/include/hih_controller/cop_boundary_defaults.hpp`에 있습니다. 각 노드의 ON/OFF와 키 조작은 실행 시 표시되는 안내를 따릅니다.

Follower admittance를 별도 터미널에서 조작하려면 기본 launch에서 중복 실행을 끕니다.

```bash
ros2 launch hih_cmd cooperation_launch.py embedded_admittance:=false
```

다른 터미널에서 실행합니다.

```bash
ros2 run hih_controller follower_admittance_controller --ros-args -p auto_approach:=true
```

`o`: admittance 토글, `p`: OFF/passthrough, `u/j`: 목표 normal force 증감, `b`: adaptive preload 토글입니다.

## 5. 외란·바람·plot·기록

High-fidelity 설정은 `src/hih_plant/config/cooperation_high_fidelity.yaml`, 외란·바람 설정은 `src/hih_plant/config/disturbances.yaml`에서 관리합니다. 모터·서보·센서 모델의 상세 기본값은 `plant/high_fidelity.py`에 있습니다.

다음 명령은 별도 터미널에서 각각 실행합니다. 외란 노드는 OFF로 시작합니다.

```bash
ros2 run hih_plant hih_payload_disturbance --ros-args --params-file src/hih_plant/config/disturbances.yaml
```

```bash
ros2 run hih_plant hih_wind_disturbance --ros-args --params-file src/hih_plant/config/disturbances.yaml
```

Payload 노드의 `c`는 CoM 이동, `r`은 CoM 복귀, `u/i/o/j/k/l`은 6축 외력·모멘트 토글, `p`는 step/sine 전환입니다. Wind 노드의 `w`로 바람을 토글합니다.

기체에 직접 외력을 입력하려면 `ros2 run hih_cmd hih_external_wrench_cmd --ros-args -r __ns:=/leader`를 사용합니다. `/follower` namespace로도 실행할 수 있습니다.

실시간 plot은 다음 명령으로 실행합니다.

```bash
ros2 run hih_plant hih_topic_plot
```

외력 전용 화면은 `ros2 run hih_plant hih_external_wrench_plot`입니다. 기본 기록 위치는 `src/hih_logging/recordings/`이며, 기록된 CSV는 `matlabscripts/HandinHand_plot.m`에서도 읽을 수 있습니다.

## 6. 기존 HandinHand_rviz 연결

시뮬레이션을 실행하고 `HIH_sim/install/setup.bash`를 source한 뒤, 기존 `HandinHand_rviz`가 빌드된 workspace의 `install/local_setup.bash`를 source합니다. `local_setup.bash`를 사용하면 RViz를 빌드할 때 기록된 이전 시뮬레이터 underlay를 다시 불러오는 일을 피할 수 있습니다.

```bash
ros2 launch hih_rviz visualization.launch.py
```

두 시뮬레이터 workspace를 동시에 source하지 말고, 기존 `HandinHand_sim` 대신 새 `HIH_sim`을 사용합니다. ROS 패키지 이름은 호환성을 위해 `hih_*` 그대로 유지합니다. RViz가 읽는 모델 경로도 `hih_plant/xml/Cooperation.xml`입니다.

| 입력 | 의미 |
|---|---|
| `/{role}/minitrone/state` | 기체 위치·자세·속도·센서·서보 상태 |
| `/{role}/minitrone/cmd`, `cmd_admittance` | 위치 reference와 admittance offset |
| `/{role}/minitrone/cop_hat`, `cop_solver_real` | MoB 및 solver CoP |
| `/{role}/minitrone/external_wrench_hat_second_order`, `contact_wrench_real` | 추정 wrench와 접촉 wrench |
| `/{role}/minitrone/contact_diagnostics` | RViz 접촉점 위치·normal force·활성 constraint |
| `/payload/contact_wrench_from_leader`, `contact_wrench_from_follower`, `contact_wrench_from_drones` | 화물 접촉 wrench |
| `/hih/ground_truth/payload/odometry`, `acceleration`, `com` | 화물 상태와 실제 CoM |
| `/hih/disturbance/payload/com_visualization_enabled` | CoM 표시 허용 heartbeat |

`{role}`은 `leader` 또는 `follower`입니다. `ContactDiagnostics`의 메시지 형식은 RViz 호환을 위해 유지하며, 사용하지 않는 마찰·slip 진단 수치는 NaN으로 둡니다. 기체 가속도는 Body IMU specific force, payload acceleration은 World linear acceleration입니다. RViz의 기본 measured-state 모드를 사용하며, 기체 ground-truth 모드는 별도 기체 GT 발행기가 있어야 합니다.
