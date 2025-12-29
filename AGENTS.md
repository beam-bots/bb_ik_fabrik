<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

BB.IK.FABRIK is a FABRIK-based inverse kinematics solver for the Beam Bots robotics framework. It computes joint angles needed to position an end-effector at a target location and orientation.

**FABRIK** (Forward And Backward Reaching Inverse Kinematics) is an iterative algorithm that works by alternately reaching from the end-effector toward the target, then from the base back to maintain segment lengths. This implementation extends classic FABRIK with orientation tracking at each joint frame.

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
  - `solve/5` - Solve IK, returns joint positions map
  - `solve_and_update/5` - Solve and update `BB.Robot.State` in-place

- **BB.IK.FABRIK.Chain** (`lib/bb/ik/fabrik/chain.ex`) - Extracts kinematic chain from `BB.Robot`:
  - Builds point/segment representation from robot topology
  - Converts solved frames back to joint angles via `frames_to_positions/4`
  - Handles co-located joints (spherical shoulders/wrists) via orientation-based angle extraction
  - Handles joint limit clamping

- **BB.IK.FABRIK.Math** (`lib/bb/ik/fabrik/math.ex`) - Pure Nx FABRIK implementation:
  - `fabrik/5` - Position-only solving
  - `fabrik_with_orientation/7` - Full frame-based solving with orientation support
  - Works on Nx tensors for performance
  - Forward/backward reaching iterations
  - Handles unreachable targets gracefully

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
- `reached` - Boolean, true if converged

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
- Arms with co-located joints (spherical shoulders/wrists) - orientation-based angle extraction

**Limited support:**
- 6-DOF anthropomorphic arms (e.g., WidowX, Kinova) - converges in point-space but may not find kinematically valid configurations
- Arms starting in mostly-vertical configurations

### Known Limitations

1. **No joint axis constraints** - FABRIK moves points to satisfy distance constraints without fully respecting joint rotation axes. The algorithm may find geometrically valid point configurations that don't correspond to achievable robot poses. This is why complex 6-DOF arms often fail despite FABRIK reporting low residuals.
2. **Mostly-vertical configurations** - Arms that start nearly vertical have poor convergence because FABRIK tends to bend end-effector joints rather than shoulder/elbow.
3. **Collinear targets** - FABRIK struggles when target is on the same line as a straight chain
4. **Serial chains only** - Does not support branching topologies
5. **Orientation solving is heuristic** - Orientation targets converge via frame propagation, which may not find optimal solutions for all arm geometries.

For 6-DOF arms requiring precise orientation control, consider analytical IK or Jacobian-based methods.

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
