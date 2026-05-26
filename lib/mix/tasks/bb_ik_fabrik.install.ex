# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbIkFabrik.Install do
    @shortdoc "Installs BB.IK.FABRIK into a project"
    @moduledoc """
    #{@shortdoc}

    The FABRIK solver is passed per-call via the `:solver` option to
    `BB.Motion` functions or `BB.Command.MoveTo` entries, so this
    installer prints a usage snippet rather than editing the topology.

    ## Example

    ```bash
    mix igniter.install bb_ik_fabrik
    ```
    """

    use Igniter.Mix.Task

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{}
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      Igniter.add_notice(igniter, notice())
    end

    defp notice do
      """
      bb_ik_fabrik: pass `solver: BB.IK.FABRIK` to BB.Motion calls or MoveTo commands.

      Ad-hoc:

          BB.Motion.move_to(MyRobot, :gripper, target, solver: BB.IK.FABRIK)

      In the DSL:

          command :go_to, BB.Command.MoveTo,
            link: :gripper,
            solver: BB.IK.FABRIK

      For continuous target tracking, see BB.IK.FABRIK.Tracker — start_link/1
      into your supervision tree with a target link and update rate.
      """
    end
  end
else
  defmodule Mix.Tasks.BbIkFabrik.Install do
    @shortdoc "Installs BB.IK.FABRIK into a project"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb_ik_fabrik.install task requires igniter. Please install igniter and try again.

          mix igniter.install bb_ik_fabrik
      """)

      exit({:shutdown, 1})
    end
  end
end
