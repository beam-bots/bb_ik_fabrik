# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.Chain do
  @moduledoc false

  alias BB.Error.Kinematics.FABRIK.UnsupportedJoint
  alias BB.Error.Kinematics.NoDofs
  alias BB.Error.Kinematics.NotAnAncestor
  alias BB.Error.Kinematics.UnknownLink
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot
  alias BB.Robot.{Joint, Kinematics}

  defstruct [:joint_names, :root_point, :root_transform, :reach, :kinematics]

  @type t :: %__MODULE__{
          joint_names: [atom()],
          root_point: Nx.Tensor.t(),
          root_transform: Transform.t(),
          reach: float(),
          kinematics: map()
        }

  @doc """
  Build a kinematic chain from a robot topology for IK solving.

  Extracts the serial chain from root to the target link, computing
  joint positions in base frame and segment lengths.

  ## Parameters

  - `robot` - The BB.Robot struct
  - `configurations` - Current joint configurations as a map
  - `source_link` - The link the chain starts at
  - `target_link` - The end-effector link name

  ## Returns

  - `{:ok, chain}` - Successfully built chain
  - `{:error, %UnknownLink{}}` - Source or target link not found; `:role` says which
  - `{:error, %NotAnAncestor{}}` - Source link does not sit above the target
  - `{:error, %NoDofs{}}` - Chain has no movable joints
  - `{:error, %UnsupportedJoint{}}` - Chain contains a `:planar` or `:floating` joint
  """
  @spec build(Robot.t(), %{atom() => Kinematics.configuration()}, atom(), atom()) ::
          {:ok, t()}
          | {:error, UnknownLink.t() | NotAnAncestor.t() | NoDofs.t() | UnsupportedJoint.t()}
  def build(%Robot{} = robot, configurations, source_link, target_link)
      when is_map(configurations) do
    with {:ok, path} <- Robot.path_between(robot, source_link, target_link),
         joints = chain_joints(robot, path),
         :ok <- reject_multi_dof(joints, source_link, target_link),
         {:ok, movable} <- movable_joints(joints, target_link, length(path)) do
      chain =
        build_chain_data(
          robot,
          configurations,
          Enum.map(movable, & &1.joint),
          Enum.map(movable, & &1.name),
          target_link
        )

      {:ok, %{chain | kinematics: build_kinematics(joints, configurations)}}
    end
  end

  @doc """
  Everything `BB.Robot.Kinematics.Defn.link_transforms/9` needs to walk this
  chain, plus the row indices that map its output back to joints and points.

  One row per link, root-first so a link's parent always precedes it: row 0 is
  the source link, carrying an identity joint, and row `j + 1` is the child of
  the chain's `j`th joint. Fixed joints get a row too — their motion is the
  identity, which is what lets a target link sitting past one be just another
  row rather than a special case.

  - `:joint_rows` - row per movable joint, in `joint_names` order
  - `:point_rows` - rows used as FABRIK points (source link, each movable joint's
    child, and the target link), with consecutive duplicates collapsed
  - `:limits_lower` / `:limits_upper` - per movable joint, infinite where unset
  """
  @spec kinematics(t()) :: map()
  def kinematics(%__MODULE__{kinematics: kinematics}), do: kinematics

  defp build_kinematics(path_joints, configurations) do
    rows = [nil | Enum.map(path_joints, & &1.joint)]
    names = [nil | Enum.map(path_joints, & &1.name)]
    row_count = length(rows)

    joint_rows =
      path_joints
      |> Enum.with_index(1)
      |> Enum.filter(fn {%{joint: joint}, _row} -> Joint.movable?(joint) end)
      |> Enum.map(fn {_joint, row} -> row end)

    movable = Enum.map(joint_rows, &Enum.at(rows, &1))

    %{
      positions: tensor(Enum.zip(rows, names), &row_position(&1, configurations)),
      origin_rpy: tensor(rows, &row_origin(&1, :orientation)),
      origin_xyz: tensor(rows, &row_origin(&1, :position)),
      axes: tensor(rows, &row_axis/1),
      is_revolute: tensor(rows, &row_mask(&1, [:revolute, :continuous])),
      is_prismatic: tensor(rows, &row_mask(&1, [:prismatic])),
      stored: Nx.broadcast(Nx.eye(4, type: :f64), {row_count, 4, 4}),
      deltas: Nx.broadcast(Nx.tensor(0.0, type: :f64), {row_count, 6}),
      parent_idx: Nx.tensor([0 | Enum.to_list(0..(row_count - 2))], type: :s32),
      joint_rows: Nx.tensor(joint_rows, type: :s32),
      point_rows: Nx.tensor(point_rows(joint_rows, row_count), type: :s32),
      limits_lower: tensor(movable, &limit(&1, :lower, :neg_infinity)),
      limits_upper: tensor(movable, &limit(&1, :upper, :infinity)),
      target_row: Nx.tensor(row_count - 1, type: :s32)
    }
  end

  @doc """
  Read a solved position tensor back into a map of joint name to value.

  The inverse of the `:positions` row the chain description hands to the solver:
  only this chain's movable joints appear, in `joint_names` order.
  """
  @spec configurations(t(), Nx.Tensor.t()) :: %{atom() => float()}
  def configurations(%__MODULE__{} = chain, positions) do
    chain.kinematics.joint_rows
    |> Nx.to_flat_list()
    |> Enum.zip(chain.joint_names)
    |> Map.new(fn {row, name} -> {name, Nx.to_number(positions[row])} end)
  end

  @doc """
  Drop the chain's joint limits, for a solve asked not to respect them.
  """
  @spec without_limits(map()) :: map()
  def without_limits(kinematics) do
    %{
      kinematics
      | limits_lower: Nx.broadcast(:neg_infinity, Nx.shape(kinematics.limits_lower)),
        limits_upper: Nx.broadcast(:infinity, Nx.shape(kinematics.limits_upper))
    }
  end

  defp tensor(items, fun), do: items |> Enum.map(fun) |> Nx.tensor(type: :f64)

  # The source link and the last movable joint's child collapse into the target
  # link's row whenever they are the same link, and a point repeated in place
  # would give the backward pass a zero-length segment to take a direction from.
  defp point_rows(joint_rows, row_count) do
    ([0] ++ joint_rows ++ [row_count - 1])
    |> Enum.dedup()
  end

  defp row_position({nil, _name}, _configurations), do: 0.0

  defp row_position({_joint, name}, configurations) do
    case Map.get(configurations, name, 0.0) do
      value when is_number(value) -> value * 1.0
      _ -> 0.0
    end
  end

  defp row_origin(nil, _key), do: [0.0, 0.0, 0.0]

  defp row_origin(%Joint{origin: origin}, key) do
    case origin do
      %{^key => {a, b, c}} -> [a, b, c]
      _ -> [0.0, 0.0, 0.0]
    end
  end

  defp row_axis(nil), do: [0.0, 0.0, 1.0]

  defp row_axis(%Joint{axis: axis}) do
    {x, y, z} = axis || {0.0, 0.0, 1.0}
    [x, y, z]
  end

  defp row_mask(nil, _types), do: 0.0
  defp row_mask(%Joint{type: type}, types), do: if(type in types, do: 1.0, else: 0.0)

  # An unlimited joint gets an infinite bound rather than a sentinel, so clamping
  # is the same arithmetic whether or not a limit was declared.
  defp limit(%Joint{limits: limits}, key, unlimited) do
    case limits do
      %{^key => value} when is_number(value) -> value
      _ -> unlimited
    end
  end

  defp chain_joints(robot, path) do
    path
    |> Enum.filter(&Map.has_key?(robot.joints, &1))
    |> Enum.map(&%{name: &1, joint: Map.fetch!(robot.joints, &1)})
  end

  # Refusing beats approximating. FABRIK repositions each joint as a point joined
  # by a fixed-length link, and a multi-DoF joint is neither — silently ignoring
  # its extra degrees of freedom would produce an answer indistinguishable from a
  # correct one.
  defp reject_multi_dof(joints, source_link, target_link) do
    case Enum.find(joints, &(&1.joint.type in [:planar, :floating])) do
      nil ->
        :ok

      %{name: name, joint: joint} ->
        {:error,
         UnsupportedJoint.exception(
           joint: name,
           joint_type: joint.type,
           source_link: source_link,
           target_link: target_link
         )}
    end
  end

  defp movable_joints(joints, target_link, path_length) do
    case Enum.filter(joints, &Joint.movable?(&1.joint)) do
      [] -> {:error, NoDofs.exception(target_link: target_link, chain_length: path_length)}
      movable -> {:ok, movable}
    end
  end

  defp build_chain_data(robot, positions, joints, joint_names, target_link) do
    transforms = Kinematics.all_link_transforms(robot, positions)
    root_link = List.first(joints).parent_link

    # How far the target link can get from the chain root: the chain laid out
    # straight, measured link origin to link origin. Anything further away is
    # beyond the arm however its joints are arranged.
    points =
      ([root_link] ++ Enum.map(joints, & &1.child_link) ++ [target_link])
      |> Enum.map(&Transform.get_translation(transforms[&1]))

    reach =
      points
      |> Enum.chunk_every(2, 1, :discard)
      |> Enum.map(fn [from, to] -> to |> Vec3.subtract(from) |> Vec3.magnitude() end)
      |> Enum.sum()

    %__MODULE__{
      joint_names: joint_names,
      root_point: points |> List.first() |> Vec3.tensor(),
      root_transform: transforms[root_link],
      reach: reach,
      kinematics: nil
    }
  end
end
