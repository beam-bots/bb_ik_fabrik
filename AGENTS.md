<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

BB.IK.FABRIK is a FABRIK-based inverse kinematics solver for the Beam Bots robotics framework. It computes joint angles needed to position an end-effector at a target location.

**FABRIK** (Forward And Backward Reaching Inverse Kinematics) is an iterative algorithm that works by alternately reaching from the end-effector toward the target, then from the base back to maintain segment lengths.

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
    │
    └── BB.IK.FABRIK.Math (pure Nx FABRIK algorithm)
```

### Key Modules

- **BB.IK.Solver** (in `bb` core) - Behaviour defining the IK solver interface. Allows pluggable solvers.

- **BB.IK.FABRIK** (`lib/bb/ik/fabrik.ex`) - Main public API implementing `BB.IK.Solver`. Entry points:
  - `solve/5` - Solve IK, returns joint positions map
  - `solve_and_update/5` - Solve and update `BB.Robot.State` in-place

- **BB.IK.FABRIK.Chain** (`lib/bb/ik/fabrik/chain.ex`) - Extracts kinematic chain from `BB.Robot`:
  - Builds point/segment representation from robot topology
  - Converts solved points back to joint angles
  - Handles joint limit clamping

- **BB.IK.FABRIK.Math** (`lib/bb/ik/fabrik/math.ex`) - Pure Nx FABRIK implementation:
  - Works on Nx tensors for performance
  - Forward/backward reaching iterations
  - Handles unreachable targets gracefully

### Usage Example

```elixir
robot = MyRobot.robot()
{:ok, state} = BB.Robot.State.new(robot)

# Solve for end-effector to reach target
target = {0.3, 0.2, 0.1}

case BB.IK.FABRIK.solve(robot, state, :end_effector, target) do
  {:ok, positions, meta} ->
    BB.Robot.State.set_positions(state, positions)
    IO.puts("Solved in #{meta.iterations} iterations, residual: #{meta.residual}")

  {:error, :unreachable, meta} ->
    IO.puts("Target unreachable, best residual: #{meta.residual}")
end
```

### Solver Options

- `:max_iterations` - Maximum FABRIK iterations (default: 50)
- `:tolerance` - Convergence tolerance in metres (default: 1.0e-4)
- `:respect_limits` - Clamp joint values to limits (default: true)

### Return Values

All results include a `meta` map with:
- `iterations` - Number of FABRIK iterations performed
- `residual` - Distance from end-effector to target (metres)
- `reached` - Boolean, true if converged
- `reason` - `:converged`, `:unreachable`, `:max_iterations`, `:unknown_link`, or `:no_dofs`

### Known Limitations

1. **Position-only** - Currently solves for position, not orientation
2. **Collinear targets** - FABRIK struggles when target is on the same line as a straight chain
3. **Serial chains only** - Does not support branching topologies

### Dependencies

- `bb` - Beam Bots core framework (uses `BB.Robot`, `BB.Robot.Kinematics`, `BB.Robot.Transform`)
- `nx` - Numerical computing for tensor operations

### Testing

Test robots are defined in `test/support/test_robots.ex`:
- `TwoLinkArm` - Simple 2-DOF planar arm
- `ThreeLinkArm` - 3-DOF arm with vertical reach
- `FixedOnlyChain` - For testing error cases

Tests use realistic off-axis targets since FABRIK works best with non-collinear geometry.
