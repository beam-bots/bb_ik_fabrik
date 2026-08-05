# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.MathTest do
  use ExUnit.Case, async: true

  alias BB.IK.FABRIK.Chain
  alias BB.IK.FABRIK.Math
  alias BB.IK.TestRobots.ThreeLinkArm
  alias BB.Robot.Kinematics

  describe "solve_constrained/3" do
    setup do
      robot = ThreeLinkArm.robot()
      zero = %{joint1: 0.0, joint2: 0.0, joint3: 0.0}
      {:ok, chain} = Chain.build(robot, zero, :base_link, :tip)

      # base_link is the robot's root, so the chain frame is the world frame and
      # targets need no conversion.
      target = fn configurations ->
        {x, y, z} = Kinematics.link_position(robot, configurations, :tip)

        %{
          position: Nx.tensor([x, y, z], type: :f64),
          rotation: Nx.eye(3, type: :f64),
          enforce: Nx.tensor(0.0, type: :f64)
        }
      end

      opts = fn tolerance, max_iterations ->
        %{
          max_iterations: max_iterations,
          tolerance: tolerance,
          orientation_tolerance: 0.01,
          lever: tolerance / 0.01
        }
      end

      %{chain: chain, kinematics: Chain.kinematics(chain), target: target, opts: opts}
    end

    # The point of constraining the solve: every pose it considers is one the
    # robot can actually hold, so it can land on the configuration that generated
    # the target rather than on a fit to something unreachable.
    test "recovers the configuration that produced the target", ctx do
      truth = %{joint1: 0.3, joint2: 0.4, joint3: 0.5}

      {solved, _iterations, residual, _orientation_residual} =
        Math.solve_constrained(ctx.kinematics, ctx.target.(truth), ctx.opts.(1.0e-9, 500))

      assert Nx.to_number(residual) <= 1.0e-9

      for {joint, value} <- Chain.configurations(ctx.chain, solved) do
        assert_in_delta value, Map.fetch!(truth, joint), 1.0e-4
      end
    end

    test "keeps every joint inside its limits", ctx do
      # Behind and below the base, so the fit wants angles the arm cannot hold.
      behind = %{
        position: Nx.tensor([-0.3, -0.2, -0.3], type: :f64),
        rotation: Nx.eye(3, type: :f64),
        enforce: Nx.tensor(0.0, type: :f64)
      }

      {solved, _iterations, _residual, _orientation_residual} =
        Math.solve_constrained(ctx.kinematics, behind, ctx.opts.(1.0e-4, 200))

      configurations = Chain.configurations(ctx.chain, solved)
      lower = Nx.to_flat_list(ctx.kinematics.limits_lower)
      upper = Nx.to_flat_list(ctx.kinematics.limits_upper)

      for {{_joint, value}, index} <- Enum.with_index(configurations) do
        assert value >= Enum.at(lower, index) - 1.0e-9
        assert value <= Enum.at(upper, index) + 1.0e-9
      end
    end

    # The gait case: one call, a lane per leg, each converging at its own rate.
    test "solves a batch of chains against their own targets", ctx do
      truths = [
        %{joint1: 0.3, joint2: 0.4, joint3: 0.5},
        %{joint1: -0.6, joint2: 0.9, joint3: -0.3},
        %{joint1: 0.1, joint2: -0.5, joint3: 0.8}
      ]

      targets = Enum.map(truths, ctx.target)

      batched_target = %{
        position: Nx.vectorize(Nx.stack(Enum.map(targets, & &1.position)), :leg),
        rotation: Nx.eye(3, type: :f64),
        enforce: Nx.tensor(0.0, type: :f64)
      }

      batched = %{
        ctx.kinematics
        | positions:
            Nx.vectorize(
              Nx.tensor(List.duplicate(Nx.to_flat_list(ctx.kinematics.positions), 3), type: :f64),
              :leg
            )
      }

      {solved, _iterations, residual, _orientation_residual} =
        Math.solve_constrained(batched, batched_target, ctx.opts.(1.0e-4, 500))

      for r <- Nx.devectorize(residual) |> Nx.to_flat_list() do
        assert r <= 1.0e-4
      end

      # A lane must be worth exactly what the same chain is worth on its own.
      devectorised = Nx.devectorize(solved)

      for {target, leg} <- Enum.with_index(targets) do
        {alone, _iterations, _residual, _orientation_residual} =
          Math.solve_constrained(ctx.kinematics, target, ctx.opts.(1.0e-4, 500))

        assert Nx.all_close(devectorised[leg], alone) |> Nx.to_number() == 1,
               "leg #{leg} disagrees with the same chain solved on its own"
      end
    end
  end
end
