<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# BB.IK.FABRIK Usage Rules

`bb_ik_fabrik` provides `BB.IK.FABRIK`, a FABRIK (Forward And Backward Reaching
Inverse Kinematics) implementation of the `BB.IK.Solver` behaviour for
[Beam Bots](https://hexdocs.pm/bb). For BB framework basics, see `bb`'s rules
(`mix usage_rules.sync <file> bb:all`); this file covers only what's specific to
the solver.

## Core principles

1. **A solver is a module you pass, not a component you declare.** `BB.IK.FABRIK`
   is not wired into the `topology` and is not supervised — hand it to a motion
   call via the `:solver` option (or use the pre-configured `BB.IK.FABRIK.Motion`
   wrapper).
2. **FABRIK is a geometric, iterative solver.** It reaches the target by walking
   the chain forwards and backwards — no Jacobian, no matrix inversion. It's fast
   and well-behaved on serial chains. Prefer `BB.IK.DLS` instead when you need
   robustness right at kinematic singularities.
3. **Solver options are an untyped keyword list with FABRIK-specific defaults.**
   There is no schema, and defaults differ from other solvers — FABRIK has no
   damping knobs, and its `:max_iterations` default is `50`.
4. **FABRIK handles single-DoF joints only.** `:revolute`, `:continuous`,
   `:prismatic` and `:fixed` are fine. A chain containing a `:planar` or
   `:floating` joint is **refused** with
   `BB.Error.Kinematics.FABRIK.UnsupportedJoint` — the algorithm treats each joint
   as a point joined to the next by a fixed-length link, and a multi-DoF joint is
   neither. Refusing beats approximating: an answer that silently dropped three or
   six degrees of freedom would look exactly like a correct one.
5. **That limit is about the chain, not the robot.** A legged robot whose body
   floats is still a FABRIK problem — solve each leg from `:body` to the foot and
   the chain contains nothing but revolute joints. `:source_link` is required and
   has no default precisely so such a chain is expressible. When the multi-DoF
   joint genuinely must be in the chain, use `BB.IK.DLS`.

## Using it

Ad-hoc, through `BB.Motion`:

```elixir
BB.Motion.move_to(MyRobot.Robot, :gripper, {0.4, 0.2, 0.1},
  source_link: :base_link,
  solver: BB.IK.FABRIK
)
```

Declaratively, as a `BB.Command.MoveTo` entry in the DSL:

```elixir
command :reach, BB.Command.MoveTo,
  link: :gripper,
  solver: BB.IK.FABRIK
```

Or via the convenience wrapper, which is `BB.Motion` with the solver pre-set:

```elixir
BB.IK.FABRIK.Motion.move_to(MyRobot.Robot, :gripper, {0.4, 0.2, 0.1})
```

Targets are `{x, y, z}` / `BB.Math.Vec3.t()` for position only, `{vec3,
orientation}` to constrain orientation, or a `BB.Math.Transform.t()`.

## Options

Passed through the motion call; all optional.

| Option | Default | Meaning |
|---|---|---|
| `:max_iterations` | `50` | Iteration cap (note: `BB.IK.DLS` defaults to `100`) |
| `:tolerance` | `1.0e-4` | Position convergence, metres |
| `:orientation_tolerance` | `0.01` | Orientation convergence, radians |
| `:respect_limits` | `true` | Clamp the solution to joint limits |

## Solving directly

For reachability checks without moving the robot, call the behaviour function.
It takes the compiled `%BB.Robot{}` struct (from `MyRobot.Robot.robot()`) and a
state or positions map — not the robot module. FABRIK distinguishes an
out-of-reach target from a failure to converge:

```elixir
robot = MyRobot.Robot.robot()
{:ok, state} = BB.Robot.State.new(robot)

case BB.IK.FABRIK.solve(robot, state, :gripper, {0.4, 0.2, 0.1}) do
  {:ok, positions, meta} -> {positions, meta.iterations}
  {:error, %BB.Error.Kinematics.Unreachable{residual: r}} -> {:out_of_reach, r}
  {:error, %BB.Error.Kinematics.NoSolution{residual: r}} -> {:no_convergence, r}
end
```

## Continuous tracking

To follow a moving target, run `BB.IK.FABRIK.Tracker` (a GenServer) in your
supervision tree; push new targets with `update_target/2`:

```elixir
{:ok, pid} = BB.IK.FABRIK.Tracker.start_link(
  robot: MyRobot.Robot, target_link: :gripper, initial_target: {0.3, 0.2, 0.1}, update_rate: 30
)
BB.IK.FABRIK.Tracker.update_target(pid, {0.35, 0.25, 0.15})
```

## Anti-patterns

- **Don't declare the solver in `topology`.** It is not a `BB.Sensor`/
  `BB.Actuator`/`BB.Controller` — there is no process to supervise. Pass
  `solver: BB.IK.FABRIK` per call.
- **Don't reach for DLS's damping options.** FABRIK has no `:lambda` or
  `:adaptive_damping`; its `:max_iterations` default is `50`, not `100`.
- **Don't pass the robot module to `solve/6`.** It wants the `%BB.Robot{}`
  struct plus a state/positions map.

## Further reading

- [bb_ik_fabrik docs](https://hexdocs.pm/bb_ik_fabrik)
- `bb`'s kinematics rules (`bb:kinematics`) and
  [Inverse Kinematics](https://hexdocs.pm/bb/09-inverse-kinematics.html)
