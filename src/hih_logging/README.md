# Logging and MATLAB Workflow

## 1. Record a run

Recording is enabled by default in the main launch.

Stop the simulation with `Ctrl+C`. Do not close the terminal until both messages appear:

```text
Bag finalized: ...
CSV converted from bag: ...
```

## 2. Output layout

Each recorder session creates a unique directory:

```text
recordings/
└── run_YYYYMMDD_HHMMSS_microseconds/
    ├── all_topics.csv
    └── rosbag/
        ├── metadata.yaml
        └── *.db3
```

## 3. Open the result in MATLAB

### Use the supplied plot script

Start MATLAB in the workspace root, add the script directory to the MATLAB path, and pass the CSV file to `HandinHand_plot`:

```matlab
addpath(fullfile(pwd, 'matlabscripts'));
HandinHand_plot('/absolute/path/to/run_.../all_topics.csv');
```

If recordings are stored in the default directory, the script can automatically select the newest run:

```matlab
addpath(fullfile(pwd, 'matlabscripts'));
HandinHand_plot();
```

The script creates figures for:

- measured Leader/Follower positions and position commands;
- normal contact force and the Follower preload target;
- Leader/Follower momentum-observer force estimates;
- Follower admittance blend;
- requested, actuated, and estimated six-axis wrenches; and
- motor and servo commands.

Missing optional topics are skipped, so the script can still open a run in which an optional controller was not active.

## 4. Add a recorded topic

1. Import the ROS message class in `hih_logging/topics.py`.
2. Add the full topic name and ROS type string to `TOPICS`.
3. Rebuild and source the logging package.
4. Record a short run and confirm that the topic appears in both the bag and CSV.
5. Update the MATLAB analysis only if the new signal is required by a plot.

```bash
colcon build --symlink-install --packages-select hih_logging
source install/setup.bash
```

The recorder is subscription-only: adding a topic cannot pass commands or measurements between the Leader and Follower.
