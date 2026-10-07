[Readme](../../README.md)

- [Log Analysis](#log-analysis)
  - [Log Analysis Tools](#log-analysis-tools)
  - [Playback](#playback)
    - [How Playback Scenarios Work](#how-playback-scenarios-work)
    - [Define a Playback Scenario](#define-a-playback-scenario)
      - [Find Topics to Exclude](#find-topics-to-exclude)
    - [Run the Scenario under Development](#run-the-scenario-under-development)
      - [Playback the Log](#playback-the-log)
    - [Select Another Scenario](#select-another-scenario)

# Log Analysis
## Log Analysis Tools
[Robot Framework ROS2 Log Analysis Tools](https://github.com/fastrobotics/robot_framework_ros2/blob/master/Tools/LogAnalysis/doc/LogAnalysis.md#log-analysis)

## Playback
--> Add note to robot_framework_ros2 Setup for Playback
This guide describes how to record a ROS 2 bag and run selected analysis nodes
against it. Playback scenarios let each analysis workflow use its own deployment
map instead of starting the live robot deployment.

### How Playback Scenarios Work

The orchestrator accepts `playback_scenario:=<name>` and loads:

```text
robot_config/robot_bringup/config/playback_scenarios/<name>/deployment_map.yaml
```

That deployment map replaces the baseline `deployment_map.yaml`; it is not
merged with the live deployment. This keeps hardware drivers and unrelated live
nodes out of an offline analysis run. The selected map still uses the normal
`host_assignments` format, and the orchestrator starts only entries assigned to
the current computer's hostname or to `*`.

Each playback scenario must have a `deployment_map.yaml`. It may also have a
`node_registry.yaml` to add scenario-specific node definitions. Nodes in the
deployment map must be present in the normal node registry or in that optional
scenario registry. Existing registered nodes can have parameters overlaid, but
their `package`, `executable`, and `launch_file` definitions cannot be changed
by a scenario registry overlay.

The orchestrator selects and starts analysis nodes; it does not start a bag
player. Run `ros2 bag play` separately in another terminal. This keeps bag
selection and playback controls independent from the nodes being tested.

### Define a Playback Scenario

Create a separate directory and complete deployment map for each analysis
workflow. For example:

```text
robot_config/robot_bringup/config/playback_scenarios/
	pose_review/
		deployment_map.yaml
	camera_review/
		deployment_map.yaml
```

Example `pose_review/deployment_map.yaml`:

```yaml
host_assignments:
  "analysis-computer":
    nodes:
      - name: "inertial_sensor_fuser_node"
      - name: "local_pose_fuser_node"
exclude_topics:
  - "/robot/fused_imu"
  - "/robot/local_pose"
  - "/robot/local_pose_angular_accel"
```

Replace `analysis-computer` with the hostname reported by `hostname` on the
computer where the orchestrator will run. These example nodes are already in
the base registry and use the configured `imu1` and `fused_imu` topics. If you
use a wildcard assignment (`"*"`), launch this scenario only on the intended
analysis computer; every computer running the orchestrator will start those
nodes. `exclude_topics` is optional and lists bag topics to suppress when
analysis nodes regenerate them. Use fully-qualified topic names from the bag;
leave out any raw input topics the analysis nodes need. Rebuild the package
after creating or changing scenario files so they are copied into the install
space.

For a node not in the base registry, add it to that scenario's optional
`node_registry.yaml`, then refer to its name in `deployment_map.yaml`. Keep
playback maps limited to software needed for analysis. Do not include motors,
servo, safety, or hardware sensor nodes in an offline playback deployment.


#### Find Topics to Exclude

With the playback nodes running, inspect their fully-qualified names and
publisher topics. `ros2 node list` shows the names to use with `ros2 node info`:

```bash
ros2 node list
ros2 node info /robot/perception/depthcamerapipeline/depthcamera_pipeline_node
ros2 bag info /mnt/usb_storage/datalogs/<bag-directory>
```

Compare the node's `Publishers` section with the topic list from `ros2 bag
info`. Exclude publisher topics that are present in the bag and are regenerated
by the playback node. Keep subscribed input topics in the bag; for this
pipeline, `/robot/front_cam/depth_registered/points` is an input, not an output
to exclude. The `perception_development` scenario excludes the pipeline's
`diagnostic`, `heartbeat`, and `ready_to_arm` publishers, plus
`/parameter_events`, in its `deployment_map.yaml`. If a listed topic is not in
the bag, excluding it has no effect.

### Run the Scenario under Development

Build and source the workspace on the analysis computer, then start the
orchestrator with the selected scenario:

```bash
cd ~/ros2_ws
colcon build --packages-select crawler_app
source install/setup.bash
ros2 launch crawler_app orchestrator.launch.py \
  robot_namespace:=robot playback_scenario:=perception_development
```


#### Playback the Log
In a second terminal on the [same ROS domain](https://github.com/fastrobotics/robot_framework_ros2/blob/master/Tools/LogAnalysis/doc/LogAnalysis.md#environment), run the playback helper:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run crawler_app playback_bag.py perception_development \
  /mnt/usb_storage/datalogs/<bag-directory>
```

The helper reads `exclude_topics` from the selected scenario's
`deployment_map.yaml`, starts `ros2 bag play` with `--clock`, and passes the
exclusions to rosbag. Excluded topics are suppressed only from the bag, so
analysis nodes remain free to publish their recomputed results. Additional
`ros2 bag play` options can be passed after the bag path, for example
`--rate 0.5` or `--remap /imu1:=/robot/imu1`.

Use the same `robot_namespace` convention that was used when recording. For
example, a node launched in namespace `robot` subscribes to `/robot/imu1` when
configured with the relative topic `imu1`. If the bag has a different topic
name, remap the playback topic or configure the analysis node to use the bag's
topic. For example, remap a root-level `/imu1` bag topic to the namespaced topic
`/robot/imu1` by passing the rosbag option after the bag path:

```bash
ros2 run crawler_app playback_bag.py perception_development /path/to/bag \
  --remap /imu1:=/robot/imu1
```

Check supported playback options with `ros2 bag play --help`.

When `playback_scenario` is set, the orchestrator automatically sets
`use_sim_time` to `true` for all ROS nodes it launches, including nodes started
through included launch files. The playback helper automatically uses `--clock`
to publish the `/clock` topic those nodes use.

Normal launches without `playback_scenario` keep their existing time settings.
For a run without simulated time, do not use `playback_scenario`; use the
general `scenario:=<name>` overlay instead if you need to change the deployment
while retaining normal time behavior.

Verify the run with:

```bash
ros2 node list
ros2 topic list
ros2 bag info /mnt/usb_storage/datalogs/<bag-directory>
```

Confirm the expected analysis nodes are running and their input topics match
the bag. Stop playback with `Ctrl+C`; stop the orchestrator separately.

### Select Another Scenario

Add another named directory under `playback_scenarios/`, with its own complete
`deployment_map.yaml`, then select it by name:

```bash
ros2 launch crawler_app orchestrator.launch.py \
	robot_namespace:=robot playback_scenario:=camera_review
```

Only one playback scenario is selected per orchestrator launch. The existing
`scenario:=<name>` argument remains available for general configuration
overlays; it is separate from `playback_scenario` and does not replace the
playback map.