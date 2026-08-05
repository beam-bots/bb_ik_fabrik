<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

BB.IK.FABRIK is a FABRIK-based inverse kinematics solver for the Beam Bots robotics framework. It computes joint angles needed to position an end-effector at a target location and orientation.

**FABRIK** (Forward And Backward Reaching Inverse Kinematics) is an iterative algorithm that works by alternately reaching from the end-effector toward the target, then from the base back to maintain segment lengths. This implementation constrains that second pass to the robot's real joint axes, so every pose it considers is one the robot can hold — see "How a solve works" below.

## Build and Test Commands

```bash
mix check --no-retry    # Run all checks (compile, test, format, credo, dialyzer, reuse)
mix test                # Run tests
mix test path/to/test.exs:42  # Run single test at line
mix format              # Format code
mix credo --strict      # Linting
```

The project uses `ex_check` - always prefer `mix check --no-retry` over running individual tools.

## Architecture

### Module Structure

```
BB.IK.Solver (behaviour, in bb core)
    ^
    | implements
BB.IK.FABRIK (public API)
    |
    ├── BB.IK.FABRIK.Chain (chain extraction from robot topology)
    ├── BB.IK.FABRIK.Math (pure Nx FABRIK algorithm)
    ├── BB.IK.FABRIK.Motion (convenience wrappers for BB.Motion)
    └── BB.IK.FABRIK.Tracker (GenServer for continuous tracking)
```

### Key Modules

- **BB.IK.Solver** (in `bb` core) - Behaviour defining the IK solver interface. Allows pluggable solvers.

- **BB.IK.FABRIK** (`lib/bb/ik/fabrik.ex`) - Main public API implementing `BB.IK.Solver`. Entry points:
  - `solve/6` - Solve IK, returns joint configurations map
  - `solve_and_update/5` - Solve and update `BB.Robot.State` in-place

- **BB.IK.FABRIK.Chain** (`lib/bb/ik/fabrik/chain.ex`) - Extracts the kinematic chain from `BB.Robot`:
  - `kinematics/1` - the tensor description the solver walks: one row per link,
    root-first, with fixed joints carrying an identity motion
  - `configurations/2` - reads a solved position tensor back into joint names
  - `reach` / `root_point` - what the reachability check needs

- **BB.IK.FABRIK.Math** (`lib/bb/ik/fabrik/math.ex`) - Pure Nx implementation:
  - `solve_constrained/3` - the solver; vectorises over a batch axis
  - `backward_pass/3` - the classic FABRIK reach it builds on, kept separate as
    the one half that needs no knowledge of how the joints may move

- **BB.IK.FABRIK.Motion** (`lib/bb/ik/fabrik/motion.ex`) - Convenience wrappers:
  - `move_to/4` - Solve and send actuator commands
  - `solve/4` - Solve without moving (validation)
  - `move_to_multi/3` - Coordinated multi-target motion (e.g., walking gaits)
  - `solve_multi/3` - Multi-target validation

- **BB.IK.FABRIK.Tracker** (`lib/bb/ik/fabrik/tracker.ex`) - GenServer for continuous tracking:
  - Periodic IK solve loop for following moving targets
  - Configurable update rate (Hz)
  - `update_target/2` - Update target position in real-time
  - `status/1` - Get current tracking status
  - `stop/1` - Stop tracking and optionally hold actuators

### Usage Example

```elixir
robot = MyRobot.robot()
{:ok, state} = BB.Robot.State.new(robot)

# Position-only target
target = Vec3.new(0.3, 0.2, 0.1)

case BB.IK.FABRIK.solve(robot, state, :end_effector, target) do
  {:ok, positions, meta} ->
    BB.Robot.State.set_positions(state, positions)
    IO.puts("Solved in #{meta.iterations} iterations, residual: #{meta.residual}")

  {:error, %BB.Error.Kinematics.Unreachable{residual: residual}} ->
    IO.puts("Target unreachable, best residual: #{residual}")

  {:error, %BB.Error.Kinematics.NoSolution{}} ->
    IO.puts("Failed to converge within max iterations")
end

# Position + orientation target (using Transform)
target = Transform.from_position_quaternion(
  Vec3.new(0.3, 0.2, 0.1),
  Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi / 4)
)

case BB.IK.FABRIK.solve(robot, state, :end_effector, target) do
  {:ok, positions, meta} ->
    IO.puts("Orientation residual: #{meta.orientation_residual} radians")

  {:error, error} ->
    IO.inspect(error)
end

# Position + axis constraint (point tool in direction)
target = {Vec3.new(0.3, 0.2, 0.1), {:axis, Vec3.new(1.0, 0.0, 0.0)}}
```

### Solver Options

- `:max_iterations` - Maximum FABRIK iterations (default: 50)
- `:tolerance` - Position convergence tolerance in metres (default: 1.0e-4)
- `:orientation_tolerance` - Orientation convergence tolerance in radians (default: 0.01)
- `:respect_limits` - Clamp joint values to limits (default: true)

### Target Types

- `Vec3.t()` - Position-only target
- `Transform.t()` - Position + full orientation (quaternion from transform)
- `{Vec3.t(), {:quaternion, Quaternion.t()}}` - Position + explicit quaternion
- `{Vec3.t(), {:axis, Vec3.t()}}` - Position + tool axis direction constraint

### Return Values

On success, returns `{:ok, positions, meta}` where `meta` contains:
- `iterations` - Number of FABRIK iterations performed
- `residual` - Distance from end-effector to target (metres)
- `orientation_residual` - Orientation error in radians (nil for position-only)
- `reached` - Boolean, whether `residual` is within `:tolerance` (and, for an
  orientation target, `orientation_residual` within `:orientation_tolerance`).
  Derived from the measured result, so `{:ok, _, %{reached: false}}` is a real
  outcome: the solve returned its best effort and it missed.

On failure, returns `{:error, error}` where error is one of:
- `%BB.Error.Kinematics.Unreachable{}` - Target beyond workspace (includes `residual`, `positions`)
- `%BB.Error.Kinematics.NoSolution{}` - Failed to converge within max iterations
- `%BB.Error.Kinematics.UnknownLink{}` - Target link not found in robot
- `%BB.Error.Kinematics.NoDofs{}` - Chain has no movable joints

### Supported Arm Configurations

**Works well:**
- 2-link planar arms (shoulder + elbow)
- 3-link arms with distinct joint positions
- SCARA-style arms
- Simple grippers with offset end-effectors
- Arms with co-located joints (spherical shoulders/wrists) - each joint is fitted
  about its own axis, so sharing a point costs nothing

### How a solve works

Classic FABRIK moves points as though every joint were a ball joint, so the
configuration it settles on generally has no counterpart in any pose the robot
can hold. This implementation keeps FABRIK's backward reach but treats its answer
as *desired directions*: the forward pass walks base to tip choosing, for each
joint, the rotation about its real world axis that carries its links closest to
those directions, clamped to its limits, and regenerates the positions beyond it
by forward kinematics.

Every pose considered is therefore one the robot can hold, and the joint values
are the solver's output rather than something recovered from a point cloud
afterwards.

Orientation rides the same fit. Three points sit rigidly on the target link's
axes, a lever arm out from it, and putting them where they are wanted is the same
as pointing the frame where it is wanted. They hang off where the end effector
*is* rather than where it is wanted, so they ask only for the turn and never
fight the real points over the reach. How loudly they argue is set each sweep by
the share of the total overshoot that orientation accounts for, each error
measured against its own tolerance — so a slack `:orientation_tolerance` hands
the sweep to position, as a caller passing one is asking for.

### Known Limitations

1. **Convergence is not guaranteed** - The solve is iterative and a minority of
   solves exhaust `:max_iterations` rather than reaching tolerance. They return
   best-effort positions with `reached: false`. Pose targets on a 6-DOF arm are
   the hardest case.
2. **Cost** - Constraining costs iterations: tens rather than the handful
   unconstrained FABRIK needs, each one a forward-kinematics pass per joint.
   Batch with `Nx.vectorize/2` when solving many chains, such as the legs of a
   gait.
3. **Collinear targets** - FABRIK struggles when the target is on the same line
   as a straight chain
4. **Serial chains only** - Does not support branching topologies

`BB.IK.DLS` remains the more reliable solver for a 6-DOF arm that must hit an
exact pose; FABRIK's draw is that it needs no Jacobian and batches cleanly.

### Dependencies

- `bb` - Beam Bots core framework (uses `BB.Robot`, `BB.Robot.Kinematics`, `BB.Math.Transform`)
- `nx` - Numerical computing for tensor operations

### Testing

Test robots are defined in `test/support/test_robots.ex`:
- `TwoLinkArm` - Simple 2-DOF planar arm
- `ThreeLinkArm` - 3-DOF arm with vertical reach
- `FixedOnlyChain` - For testing error cases (no movable joints)
- `PrismaticArm` - Arm with prismatic (linear) joint
- `ContinuousJointArm` - Arm with unlimited rotation joint
- `SixDofArm` - 6-DOF anthropomorphic arm for orientation testing

Tests use realistic off-axis targets since FABRIK works best with non-collinear geometry.

## Licensing headers

Every source file must carry an SPDX header — a `#`-style comment for code, an
HTML comment for Markdown, or a `<file>.license` sidecar for files that can't
hold comments (binaries, JSON, lockfiles). `mix check` runs `reuse lint` and
fails the build if one is missing.

When you create a new file, its `SPDX-FileCopyrightText` line must credit **the
user you are working for** — not you (the agent), and not this repo's original
author. Take their name from `git config user.name` (add their `user.email` if
you include one) and use the current year. Match the neighbouring files'
`SPDX-License-Identifier` (usually `Apache-2.0`):

```
SPDX-FileCopyrightText: <current year> <your user's name>

SPDX-License-Identifier: Apache-2.0
```

Never copy an existing file's copyright line onto a new file — that credits the
wrong person. When you only edit an existing file, leave its headers unchanged.
