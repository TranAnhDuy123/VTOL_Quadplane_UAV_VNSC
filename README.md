# VTOL QuadPlane UAV Simulation

Gazebo Sim and ArduPilot SITL simulation package for a separate-lift Hybrid VTOL fixed-wing UAV with a 4+1 configuration:

- Four vertical-lift motors
- One rear pusher motor
- Ailerons, elevator, and twin rudders
- PID baseline controller
- Experimental roll-axis INDI controller
- Position-triggered roll disturbance plugin

## Simulation Environment

The project was developed and tested with:

- Ubuntu 24.04
- Gazebo Sim 8
- ArduPilot SITL / ArduPlane
- QGroundControl
- CMake and C++17

## Repository Structure

```text
.
├── plugins/
│   └── location_triggered_disturbance/
│       ├── CMakeLists.txt
│       └── src/
│           └── LocationTriggeredWrench.cc
│
├── simulation/
│   ├── ardupilot_gazebo/
│   │   ├── config/
│   │   ├── models/
│   │   └── worlds/
│   ├── models/
│   │   ├── VTOL_Quadplane/
│   │   └── alti_transition_quad/
│   ├── gui/
│   └── terrain/
│
└── README.md
```

## Main Files

- Aircraft model:

  `simulation/models/VTOL_Quadplane/model.sdf`

- Gazebo world:

  `simulation/ardupilot_gazebo/worlds/alti_transition_runway.sdf`

- ArduPlane parameter file:

  `simulation/ardupilot_gazebo/config/alti_transition_quad.param`

- Disturbance plugin source:

  `plugins/location_triggered_disturbance/src/LocationTriggeredWrench.cc`

## Build the Location-Triggered Disturbance Plugin

From the repository root:

```bash
cmake \
  -S plugins/location_triggered_disturbance \
  -B plugins/location_triggered_disturbance/build

cmake --build \
  plugins/location_triggered_disturbance/build \
  -j"$(nproc)"
```

The generated library should be:

```text
plugins/location_triggered_disturbance/build/libLocationTriggeredWrench.so
```

## Configure Gazebo Paths

Run from the repository root:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/plugins/location_triggered_disturbance/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

export GZ_SIM_RESOURCE_PATH="$PWD/simulation/models:$PWD/simulation/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH}"
```

These commands must be executed in every new terminal session unless they are added to `.bashrc`.

## Start Gazebo

```bash
gz sim -v4 -r \
simulation/ardupilot_gazebo/worlds/alti_transition_runway.sdf
```

A successful disturbance-plugin load should include a message similar to:

```text
[LocationTriggeredWrench] Armed for link [base_link]
```

## Start ArduPlane SITL

ArduPilot must be installed separately.

Example command:

```bash
cd /path/to/ardupilot

sim_vehicle.py \
  -v ArduPlane \
  -f quadplane \
  --model JSON \
  --add-param-file=/path/to/VTOL_Quadplane_UAV_VNSC/simulation/ardupilot_gazebo/config/alti_transition_quad.param \
  --console
```

Replace `/path/to/...` with the actual paths on the local computer.

## Roll Controller Selection

The experimental parameter `RLL_INDI_EN` selects the roll controller:

| Value | Operating mode |
|---:|---|
| `0` | Conventional PID |
| `1` | PID active with INDI shadow logging |
| `2` | INDI active on the roll axis |

The custom ArduPilot INDI source or patch must be installed before this parameter becomes available.

## Position-Triggered Disturbance

The custom Gazebo system plugin applies a constant body-roll moment while the aircraft remains inside a defined three-dimensional zone.

The current test configuration is stored in:

```text
simulation/models/VTOL_Quadplane/model.sdf
```

The disturbance is activated on zone entry and deactivated when the aircraft exits the zone. The trigger remains locked until the simulation is restarted.

Example startup message:

```text
[LocationTriggeredWrench] Armed for link [base_link];
body-roll torque=1 N.m while the aircraft remains inside the zone.
```

## Test Method

The primary comparison cases are:

1. PID trajectory tracking without disturbance.
2. INDI trajectory tracking without disturbance.
3. PID flight through the fixed disturbance zone.
4. INDI flight through the same disturbance zone.

The same mission, aircraft model, parameter set, waypoint radius, airspeed command, and disturbance configuration should be used for each PID–INDI comparison.

## Runtime Files

Flight logs, Gazebo runtime data, compiled libraries, and build directories are intentionally excluded from Git. These files should be generated locally.