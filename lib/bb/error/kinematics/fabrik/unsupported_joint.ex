# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Kinematics.FABRIK.UnsupportedJoint do
  @moduledoc """
  The requested chain contains a joint FABRIK cannot solve for.

  FABRIK is not Jacobian-based. It is a heuristic that iteratively repositions
  joints along a chain, treating each as a point connected by a fixed-length link,
  and a `:planar` or `:floating` joint has no meaningful interpretation in that
  scheme — there is no single point to reposition, and no fixed length between one
  and the next.

  Refusing is better than approximating: a wrong answer from a solver that
  silently ignored three or six degrees of freedom looks exactly like a right one.

  ## This is a property of the chain, not the robot

  A legged robot whose body floats is a perfectly good FABRIK problem when each
  leg is solved from the body to the foot, because those chains contain nothing but
  revolute joints. Scope the solve past the multi-DoF joint with `source_link` and
  FABRIK handles it:

      # The floating base is not part of this problem.
      BB.IK.FABRIK.solve(robot, state, :body, :front_left_foot, target, [])

  For a chain that genuinely must include the multi-DoF joint, use a
  Jacobian-based solver — `bb_ik_dls` is dimension-agnostic and needs no special
  case.
  """
  use BB.Error,
    class: :kinematics,
    fields: [:joint, :joint_type, :source_link, :target_link]

  @type t :: %__MODULE__{
          joint: atom(),
          joint_type: :planar | :floating,
          source_link: atom() | nil,
          target_link: atom() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{joint: joint, joint_type: type, source_link: source, target_link: target}) do
    "FABRIK cannot solve chains containing #{inspect(joint)}, which is a " <>
      "#{inspect(type)} joint with #{dof(type)} degrees of freedom. " <>
      scope_hint(source, target) <>
      "or use a Jacobian-based solver such as BB.IK.DLS."
  end

  defp dof(:planar), do: "three"
  defp dof(:floating), do: "six"

  defp scope_hint(nil, _target), do: ""

  defp scope_hint(_source, target) do
    "Scope the solve below it with a `source_link` that excludes it " <>
      "(the chain to #{inspect(target)} would then contain only single-DoF joints), "
  end
end
