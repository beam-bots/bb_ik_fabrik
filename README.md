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
- Respects joint limits with optional clamping
- Uses Nx tensors for efficient computation
- Returns best-effort positions even when targets are unreachable

## Installation

Add `bb_ik_fabrik` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:bb_ik_fabrik, "~> 0.1.0"}
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

  {:error, :unreachable, meta} ->
    # Target is beyond the robot's reach
    IO.puts("Target unreachable, best distance: #{meta.residual}m")
    # meta.positions contains best-effort joint values
end
```

### Solving with Options

```elixir
BB.IK.FABRIK.solve(robot, state, :end_effector, target,
  max_iterations: 100,    # Maximum FABRIK iterations (default: 50)
  tolerance: 0.001,       # Convergence tolerance in metres (default: 1.0e-4)
  respect_limits: true    # Clamp to joint limits (default: true)
)
```

### Using solve_and_update/5

For convenience, `solve_and_update/5` solves IK and updates the state in one call:

```elixir
case BB.IK.FABRIK.solve_and_update(robot, state, :end_effector, target) do
  {:ok, positions, meta} ->
    # State has already been updated
    :ok

  {:error, reason, meta} ->
    # State is unchanged on error
    {:error, reason}
end
```

### Target Formats

Targets can be specified as:

```elixir
# Position tuple
target = {0.3, 0.2, 0.1}

# Nx tensor
target = Nx.tensor([0.3, 0.2, 0.1])

# 4x4 homogeneous transform (position extracted, orientation ignored)
target = BB.Robot.Transform.translation(0.3, 0.2, 0.1)
```

## Limitations

- **Position-only**: Currently solves for position, not orientation
- **Serial chains only**: Does not support branching topologies
- **Collinear targets**: FABRIK can struggle when the target lies on the same line as a straight chain

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_ik_fabrik).
