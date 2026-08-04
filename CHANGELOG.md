<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Change Log

All notable changes to this project will be documented in this file.
See [Conventional Commits](Https://conventionalcommits.org) for commit guidelines.

<!-- changelog -->

## [v0.6.0](https://github.com/beam-bots/bb_ik_fabrik/compare/v0.5.0...v0.6.0) (2026-08-04)
### Breaking Changes:

* refuse multi-DoF chains, and scope solves with `source_link` (#86) by James Harton



## [v0.5.0](https://github.com/beam-bots/bb_ik_fabrik/compare/v0.4.1...v0.5.0) (2026-08-03)
### Breaking Changes:

* migrate the test mock actuator to `handle_command/2` (#81) by James Harton



### Improvements:

* drive the tracking loop from `BB.Loop` (#83) by James Harton

## [v0.4.1](https://github.com/beam-bots/bb_ik_fabrik/compare/v0.4.0...v0.4.1) (2026-06-26)




### Improvements:

* express FABRIK (position + orientation) as composable defns (#60) by James Harton

## [v0.4.0](https://github.com/beam-bots/bb_ik_fabrik/compare/v0.3.2...v0.4.0) (2026-05-28)




### Features:

* add `bb_ik_fabrik.install` igniter task (#33) by James Harton

## [v0.3.2](https://github.com/beam-bots/bb_ik_fabrik/compare/v0.3.1...v0.3.2) (2026-05-17)




## [v0.3.1](https://github.com/beam-bots/bb_ik_fabrik/compare/v0.3.0...v0.3.1) (2026-01-04)




### Improvements:

* update for bb 0.12 command API and add bb_dep helper (#9) by James Harton

## [v0.3.0](https://github.com/beam-bots/bb_ik_fabrik/compare/v0.2.0...v0.3.0) (2025-12-29)




### Features:

* orientation-aware FABRIK with co-located joint handling (#6) by James Harton

* update FABRIK to use Vec3 targets and orientation types by James Harton

* add orientation-aware FABRIK solving with axis constraints by James Harton

### Improvements:

* use orientation-aware FABRIK internally for all solves by James Harton

## [v0.2.0](https://github.com/beam-bots/bb_ik_fabrik/compare/v0.1.0...v0.2.0) (2025-12-18)




### Features:

* add FABRIK solver with motion integration (#1) by James Harton

