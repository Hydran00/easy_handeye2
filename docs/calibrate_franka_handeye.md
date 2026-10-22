# IFL: Polaris to Franka base

`calibrate_franka_handeye.launch.py` estimates `polaris_frame -> robot_base_frame`
with an NDI marker rigidly attached to the Franka end effector. It does not
calibrate the probe tip. Keep both bases fixed during calibration.

Start the Franka bringup with live joint states and `robot_state_publisher`.
The calibrator reads the resulting base-to-EE TF chain at the same timestamp
as the Polaris measurement. Start the Polaris driver as well.

```bash
ros2 launch easy_handeye2 calibrate_franka_handeye.launch.py \
  robot_base_frame:=panda_link0 robot_effector_frame:=panda_link8
```

For an FR3 or prefixed robot, pass the actual TF frame names. The defaults
target a Panda. `ndi_sensor_name` defaults to `marker`, the rigid body name in
`RigidArray.frames`; its TF child is `marker`. The message reference frame
must match `polaris_frame` (default `polaris_frame`).
The input topic defaults to `/rigid_poses`; override `rigid_bodies_topic` if
your driver uses a different topic.

The GUI waits for the server's sampling services before opening. Those
services become available only when both TF chains exist. If it stays on
`Waiting for sampling services`, inspect the server log and verify:

```bash
ros2 run tf2_ros tf2_echo polaris_frame marker
ros2 run tf2_ros tf2_echo panda_link0 panda_link8
```

Use your configured Franka frame names in the second command. Receiving
`RigidArray` messages alone does not provide the Franka kinematic chain.

If the Polaris bringup already publishes `polaris_frame -> marker`, pass
`start_marker_tf:=false` to avoid duplicate broadcasters. Do not publish an
old calibration connecting the Franka and Polaris trees during acquisition.
In particular, an existing `world -> polaris_frame` publisher must be stopped
before publishing the new calibration if it would give Polaris two parents.

Move the EE through varied positions and rotations around multiple axes,
keeping the marker visible. Stop at each pose, acquire samples in rqt, then
compute and save. The calibration name is `calibrator_holder_ifl`.

The launch exchanges the mathematical roles of the two pose sources in
`eye_on_base`, so the saved transform is directly **Polaris -> Franka base**.
The rqt robot fields therefore show the Polaris chain, and the tracking
fields show the Franka chain. Publish the saved result with:

```bash
ros2 launch easy_handeye2 publish_franka_handeye.launch.py
```

Save the computed calibration in rqt before starting the publisher. Keep the
Franka and Polaris bringups running. The publisher loads the saved file and
publishes the static TF `polaris_frame -> panda_link0` (or the base frame
selected during calibration). Stop any existing TF publisher assigning a
different parent to the Franka base, such as `world -> panda_link0`.

In RViz, set Fixed Frame to `polaris_frame` and display TF and RobotModel.
Move the robot through poses not used for calibration: `marker` should stay
at a constant position and orientation relative to `panda_link8`. This checks
consistency; the marker frame need not coincide with the EE frame. Restart
the publisher after saving a new calibration to load the updated result.
