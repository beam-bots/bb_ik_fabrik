<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# BB.IK.FABRIK

[![CI](https://github.com/beam-bots/bb_ik_fabrik/actions/workflows/ci.yml/badge.svg)](https://github.com/beam-bots/bb_ik_fabrik/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/bb_ik_fabrik.svg)](https://hex.pm/packages/bb_ik_fabrik)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/bb_ik_fabrik)](https://api.reuse.software/info/github.com/beam-bots/bb_ik_fabrik)

A FABRIK-based inverse kinematics solver for the [Beam Bots](https://github.com/beam-bots/bb) robotics framework.

FABRIK (Forward And Backward Reaching Inverse Kinematics) is an iterative algorithm that computes joint angles needed to position an end-effector at a target location. It works by alternately reaching from the end-effector toward the target, then from the base back to maintain segment lengths.

## Features

- Implements the `BB.IK.Solver` behaviour for pluggable IK solvers
- Works with `BB.Robot.State` or plain position maps
- Supports revolute, prismatic, and continuous joints
- Position and orientation solving (quaternion or axis constraints)
- Respects joint limits with optional clamping
- Uses Nx tensors for efficient computation
- Returns best-effort positions even when targets are unreachable

## Installation

Add `bb_ik_fabrik` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:bb_ik_fabrik, "~> 0.2.0"}
  ]
end
```

## Usage

```elixir
# Define your robot using the BB DSL
robot = MyRobot.robot()

# Create initial state
{:ok, state} = BB.Robot.State.new(robot)

# Define a target position for the end-effector
target = {0.3, 0.2, 0.1}

# Solve inverse kinematics
case BB.IK.FABRIK.solve(robot, state, :end_effector, target) do
  {:ok, positions, meta} ->
    # Apply the solved positions to the robot state
    BB.Robot.State.set_positions(state, positions)
    IO.puts("Solved in #{meta.iterations} iterations")
    IO.puts("Final distance to target: #{meta.residual}m")

  {:error, %BB.Error.Kinematics.Unreachable{residual: residual, positions: positions}} ->
    # Target is beyond the robot's reach
    IO.puts("Target unreachable, best distance: #{residual}m")
    # positions contains best-effort joint values

  {:error, %BB.Error.Kinematics.NoSolution{}} ->
    IO.puts("Failed to converge within max iterations")
end
```

### Solving with Options

```elixir
BB.IK.FABRIK.solve(robot, state, :end_effector, target,
  max_iterations: 100,         # Maximum FABRIK iterations (default: 50)
  tolerance: 0.001,            # Position tolerance in metres (default: 1.0e-4)
  orientation_tolerance: 0.1,  # Orientation tolerance in radians (default: 0.01)
  respect_limits: true         # Clamp to joint limits (default: true)
)
```

### Using solve_and_update/5

For convenience, `solve_and_update/5` solves IK and updates the state in one call:

```elixir
case BB.IK.FABRIK.solve_and_update(robot, state, :end_effector, target) do
  {:ok, positions, meta} ->
    # State has already been updated
    :ok

  {:error, %BB.Error.Kinematics.Unreachable{} = error} ->
    # State is unchanged on error
    {:error, error}
end
```

### Target Formats

Targets can be specified as:

```elixir
# Position only (Vec3)
target = BB.Math.Vec3.new(0.3, 0.2, 0.1)

# Position with axis constraint ("point tool in this direction")
target = {BB.Math.Vec3.new(0.3, 0.2, 0.1), {:axis, BB.Math.Vec3.new(0.0, 0.0, -1.0)}}

# Position with full orientation (quaternion)
quat = BB.Math.Quaternion.from_axis_angle(BB.Math.Vec3.unit_z(), :math.pi() / 4)
target = {BB.Math.Vec3.new(0.3, 0.2, 0.1), {:quaternion, quat}}

# 4x4 homogeneous transform (extracts both position and orientation)
target = BB.Math.Transform.from_position_quaternion(
  BB.Math.Vec3.new(0.3, 0.2, 0.1),
  BB.Math.Quaternion.identity()
)
```

When using orientation constraints, the result metadata includes `orientation_residual` (in radians) alongside the position `residual`.

## Supported Arm Configurations

FABRIK works best with **simple planar or spatial arms** where:
- Joints have **significant lever arms** between them
- The arm has **2-4 degrees of freedom**

### Works Well

- **2-link planar arms**: Classic shoulder + elbow configuration
- **3-link arms**: Shoulder + elbow + wrist with distinct positions
- **SCARA-style arms**: Horizontal joints with vertical offsets
- **Simple grippers**: Where the end-effector is offset from the last joint
- **Arms with co-located joints** (spherical wrists/shoulders): Handled via orientation-based angle extraction

### Limited Support

- **6-DOF anthropomorphic arms** (e.g., WidowX, Kinova): FABRIK converges in point-space but may not find kinematically valid configurations. The algorithm distributes movement toward the end-effector rather than through shoulder/elbow rotation.

## Limitations

- **Serial chains only**: Does not support branching topologies
- **No joint axis constraints**: FABRIK moves points to satisfy distance constraints without fully respecting joint rotation axes. This means the algorithm may find geometrically valid point configurations that don't correspond to achievable robot poses.
- **Collinear targets**: FABRIK can struggle when the target lies on the same line as a straight chain
- **Mostly-vertical configurations**: Arms that start nearly vertical (like many 6-DOF arms at home position) may have poor convergence because FABRIK tends to bend the end-effector joints rather than the shoulder/elbow.
- **Orientation solving is heuristic**: Orientation targets converge via frame propagation, which may not find optimal solutions for all arm geometries.

### When to Use a Different Solver

Consider analytical IK or Jacobian-based methods when:
- You have a 6-DOF arm with specific geometry (closed-form solutions exist)
- You need precise control over which joints move
- You need to optimise for specific joint configurations (elbow up vs down)
- You require guaranteed orientation accuracy for complex arm geometries

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_ik_fabrik).
