# ArduPilot Roll INDI Controller Patch

This directory contains the ArduPilot modifications used to implement and test the experimental fixed-wing roll-axis INDI controller.

## Included Files

- `roll_indi.patch`: complete controller modifications relative to the recorded ArduPilot base commit.
- `base_commit.txt`: ArduPilot commit against which the patch was generated.
- `changed_files.txt`: list of files modified by the patch.

The patch includes both the committed development history and the latest local controller changes used during simulation testing.

## Controller Modes

The parameter `RLL_INDI_EN` selects the active roll-control configuration:

| Value | Mode |
|---:|---|
| `0` | Conventional PID controller |
| `1` | PID active with INDI shadow computation and logging |
| `2` | INDI active on the fixed-wing roll axis |

Only the fixed-wing roll controller is replaced. The remaining aircraft control loops continue using the existing ArduPlane control architecture.

## Apply the Patch

First clone ArduPilot:

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

Define the path to the cloned VTOL simulation repository:

```bash
UAV_REPO=/absolute/path/to/VTOL_Quadplane_UAV_VNSC
```

Check out the exact base commit:

```bash
INDI_BASE_COMMIT=$(tr -d '\n' < \
"$UAV_REPO/controller/ardupilot/base_commit.txt")

git checkout "$INDI_BASE_COMMIT"
git switch -c indi-fixedwing-dev
```

Verify and apply the patch:

```bash
git apply --check \
"$UAV_REPO/controller/ardupilot/roll_indi.patch"

git apply \
"$UAV_REPO/controller/ardupilot/roll_indi.patch"
```

Inspect the resulting changes:

```bash
git status --short
git diff --stat
```

## Build ArduPlane SITL

Install the ArduPilot prerequisites if they have not already been installed:

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Configure and build SITL:

```bash
./waf configure --board sitl
./waf plane
```

The build must finish successfully before starting the PID–INDI simulation comparisons.

## Important Notes

- Apply the patch only to the commit recorded in `base_commit.txt`.
- Do not apply it directly to an unrelated ArduPilot version.
- Use identical aircraft parameters, mission, disturbance zone, and initial conditions when comparing PID and INDI.
- The patch represents the current research implementation and is not an official ArduPilot controller.