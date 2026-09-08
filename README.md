# autonomous_private

A ROS (Noetic) catkin workspace for autonomous F1/10-scale racecar driving in Gazebo.
A simulated 1/10-scale car reads its 2D LIDAR, picks a steering angle and speed, and
drives itself around a race track — no human input.

Two driving strategies live here:

- **Wall following + PD control** — the classic F1/10 pipeline: a wall-follower node
  publishes a cross-track `error`, a PD controller turns that error into a steering
  angle and velocity.
- **Disparity extender** — a reactive gap-following planner that inflates obstacle
  edges by the car's half-width and steers toward the widest remaining gap. This is
  the strategy the most recent work targets (`disparity_extender.py`).

## Layout

```
src/
  f1_10_sim-master/race/        the driving code (this is what you'll edit)
    scripts/                    nodes: planners, controller, teleop, telemetry
    msg/                        drive_param, drive_values, pid_input
    launch/f1_tenth.launch      full simulation + autonomy stack
  racecar-simulator-master/     Gazebo model, worlds, ros_control config
  racecar/                      MIT racecar: ackermann_cmd_mux (submodule)
  ackermann_msgs/               AckermannDriveStamped messages (submodule)
  vesc/                         VESC driver + ackermann-to-VESC bridge
  serial/                       serial port library used by vesc
build/  devel/                  catkin build output (checked in)
```

## Nodes in the `race` package

| Script | Role |
| --- | --- |
| `disparity_extender.py` | Reactive planner. Subscribes `scan`, extends disparities, steers at the widest gap, publishes `drive_parameters`. |
| `levineDemo.py` / `levineDemo1.py` | Wall-following planners. Publish cross-track `error`. |
| `robust_obs_finder.py` | Wall follower with obstacle handling, publishes `error`. |
| `control.py` | PD controller. `error` → `drive_parameters` (angle capped ±30°, speed capped 2.5 m/s). Prompts for Kp/Ki/Kd and velocity on startup. |
| `sim_connector.py` | Bridge. `drive_parameters` → `/vesc/ackermann_cmd_mux/input/teleop` (`AckermannDriveStamped`). |
| `keyboard.py` | WASD teleop, publishes `drive_parameters`. |
| `talker.py` | Hardware path: `drive_parameters` → PWM `drive_pwm` for the real car. |
| `kill.py` | Emergency stop over the `eStop` topic (Delete = stop, Home = resume). |
| `show_speed.py` | Prints speed in km/h from `/vesc/odom`. |
| `dist_finder.py` | Skeleton wall-follower left as an exercise — `getRange` is unimplemented. |

Data flow for the autonomous run:

```
Gazebo → /scan → planner → drive_parameters → sim_connector → /vesc/ackermann_cmd_mux/input/teleop → Gazebo
```

The PD variant inserts `control.py` between the planner and `drive_parameters`,
with the planner publishing `error` instead.

## Requirements

- Ubuntu 20.04 with ROS Noetic (`ros-noetic-desktop-full`)
- Gazebo 11, `gazebo_ros`, `ros_control`, `topic_tools`, `xacro`

## Setup

`src/racecar` and `src/ackermann_msgs` are recorded as gitlinks but the repository
has no `.gitmodules`, and `src/vesc` and `src/serial` are empty. Clone them by hand
before building:

```sh
cd src
rm -rf racecar ackermann_msgs vesc serial
git clone https://github.com/mit-racecar/racecar.git
git clone https://github.com/ros-drivers/ackermann_msgs.git
git clone https://github.com/mit-racecar/vesc.git
git clone https://github.com/wjwwood/serial.git
```

Then build and source the workspace:

```sh
cd ..
catkin_make
source devel/setup.bash
```

Make sure the Python nodes are executable (`chmod +x src/f1_10_sim-master/race/scripts/*.py`).

## Running

Full stack — Gazebo on the Barca track, plus the PD controller and the sim bridge:

```sh
roslaunch race f1_tenth.launch
```

`control.py` runs with `output="screen"` and will ask for Kp, Ki, Kd and velocity in
the terminal before the car moves.

To drive with the disparity extender instead, uncomment its node in
`src/f1_10_sim-master/race/launch/f1_tenth.launch` (and comment out `control_node`,
since both publish `drive_parameters`), or just run it alongside a bare simulator:

```sh
roslaunch racecar_gazebo racecar.launch world_name:=track_barca
rosrun race sim_connector.py
rosrun race disparity_extender.py
```

Other worlds are available in `racecar_gazebo/worlds/`: `racecar`, `track_porto`,
`racecar_tunnel`, `racecar_walker`, `racecar_cones`, `racecar_parking_1`, `racecar_ar`.
Pass one via `world_name:=`.

Manual driving:

```sh
rosrun race keyboard.py        # w/a/s/d
```

## Tuning notes

- `control.py` — `kp = 10`, `kd = 0.01`, `kp_vel = 42`; steering is clamped to ±30°
  and speed is stepped down on hard turns (2.0 m/s straight, 0.8, then 0.3).
- `disparity_extender.py` — `vel = 6`, `max_vel = 12` (used when the widest gap is
  over 8 m), disparity inflation of `0.40` rad, and the outer 220 scan indices on
  each side are ignored when searching for the gap.

## Credits

Built on upstream open-source work, vendored into `src/`:

- [mlab-upenn/f1_10_sim](https://github.com/mlab-upenn/f1_10_sim)
- [mlab-upenn/racecar-simulator](https://github.com/mlab-upenn/racecar-simulator)
- [mit-racecar/racecar](https://github.com/mit-racecar/racecar) and [vesc](https://github.com/mit-racecar/vesc)
