# UR10e + Robotiq 2F-140 ManyForge Scene Contract

## Purpose

This document owns the Isaac USD asset and scene-authoring side of the
ManyForge UR10e/Robotiq integration. The runtime/operator contract is in the
ManyForge repository. Development rationale and verification evidence are in
the `manyforge_specs` repository.

The implementation has one mechanical and command authority:

1. NVIDIA's Isaac Sim 5.1 Robotiq asset owns the gripper bodies, collisions,
   one driven `finger_joint`, and five PhysX mimic joints.
2. The arm and gripper form one articulation. The gripper asset's standalone
   `root_joint` is disabled and the UR `ee_joint` attaches
   `robotiq_arg2f_base_link` to `ee_link`.
3. One OmniGraph articulation controller writes only `finger_joint`.
4. Mimic joints are observed for completion but are never commanded.

This is intentionally different from the URDF command surface. A URDF mimic
declaration describes kinematics but does not make every imported Isaac asset
enforce mimic motion. This scene uses NVIDIA's native PhysX mimic-joint model,
so commanding only the drive joint is both mechanically correct and effective.

## Asset Composition

The composition path is:

```text
Scenario/.../isaac_manipulator_scene[_light].usda
  -> Robots/ur10e_robotiq2f-140_ROS_manyforge.usda
     -> Robots/ur10e_robotiq2f-140_ROS.usd (untouched known-good layer)
     -> Robots/UniversalRobots/ur10e/ur10e_manyforge.usda
        -> Robots/UniversalRobots/ur10e/ur10e.usda (unchanged shared asset)
        -> configuration/ur10e_Gripper_2F_140_manyforge.usda
           -> NVIDIA Isaac 5.1 Robotiq HTTPS payload
```

The wrapper references NVIDIA's versioned Isaac 5.1 HTTPS asset directly.
Meshes, collisions, masses, and mimic physics stay upstream; this repository
does not duplicate heavy vendor data. The exact source URL is contract-checked,
and ManyForge overrides live only in the small wrapper layer. The official root
layer observed on 2026-07-17 had SHA-256
`ef460ed8eb17c38759685a2f5f58bb1cbc674a926ac24f48952af064811fe0a0`.

A fresh Isaac workstation therefore needs access to the NVIDIA asset endpoint
or a pre-populated Isaac resolver cache. Missing upstream content fails closed:
the six-joint topology and readiness contract cannot become ready. A deployment
that must be air-gapped should mirror this same versioned URL in controlled
artifact storage rather than committing the heavy files to this examples repo.

## Minimum Source Footprint

The final composition deliberately leaves these files untouched:

- the known-good binary `Robots/ur10e_robotiq2f-140_ROS.usd`;
- the shared `Robots/UniversalRobots/ur10e/ur10e.usda`;
- every NVIDIA mesh, collision, material, and physics file.

Four small USDA files carry the ManyForge-specific contract: the robot
sublayer, UR variant overlay, gripper attachment wrapper, and command graph.
The two existing full/light scenes are the only large source files changed.
Their diffs are mostly deletions of legacy gripper drives/graphs that are
stronger than payload opinions. A simple top-level payload overlay cannot
safely hide those scalar and API opinions; it would need a similarly long and
more fragile property-block list. Duplicating whole derived scenes or
generating them at deployment time would increase drift and review risk. Direct
cleanup of those two source layers is therefore the smallest reliable option.

`ur10e_Gripper_2F_140_manyforge.usda`:

- disables the gripper's standalone articulation root;
- attaches the gripper base through the UR `ee_joint`;
- caps the master drive at 200 N for stable 60 Hz contact behavior;
- preserves NVIDIA's masses, collision shapes, limits, and mimic laws.

`ur10e_manyforge.usda` is a small variant overlay. It keeps the shared UR asset
unchanged and substitutes the corrected Robotiq payload only for this ManyForge
composition, avoiding behavior changes in unrelated UR scene consumers.

`ur10e_robotiq2f-140_ROS_manyforge.usda` is a thin strong sublayer over the
untouched known-good binary robot stage. It replaces only the inner arm payload
and deactivates only the two legacy gripper namespaces. The arm graph, root
transform stack, timing metadata, and all unrelated robot opinions remain
authored by the original binary layer.

`robotiq_action_graph.usda`:

- polls `/isaac_gripper_state` on playback ticks;
- dispatches only when a new ROS message arrives;
- targets the complete robot articulation root;
- sends only `finger_joint` to the articulation controller;
- contains no raw USD property writer and no second controller.

The scene marker
`/World/ManyForgeContracts/RobotiqSingleAuthorityV1` identifies the expected
`one_master_drive_six_joint_settled_completion_v1` contract before live tests.

## Joint Contract

For a drive target `q`, the expected observed joints are:

| Joint | Expected position |
| --- | ---: |
| `finger_joint` | `q` |
| `left_inner_finger_joint` | `q` |
| `left_inner_knuckle_joint` | `-q` |
| `right_inner_knuckle_joint` | `-q` |
| `right_outer_knuckle_joint` | `-q` |
| `right_inner_finger_joint` | `q` |

Changing these signs or commanding follower joints creates competing
authorities and invalidates the qualification.

## Validation And Deployment

Run the parser-based contract validator before deployment:

```bash
docker exec manyforge-usd-tools \
  python3 /repo/tools/validate_ur10e_robotiq_contract.py --asset-root /repo
```

Deploy a complete immutable release to the Isaac workstation:

```bash
tools/deploy_isaac_release.sh \
  --host 192.168.1.132 \
  --identity ~/.ssh/manyforge_isaac_laptop_ed25519
```

The script validates local composition, computes a local SHA-256 manifest,
transfers to an isolated incoming directory, verifies every local file
remotely, then atomically promotes the directory under `releases/`. The
manifest covers the ManyForge layers, not NVIDIA's remotely resolved payload.
Never hot-edit the checkout or a loaded release on the Isaac workstation.

## Live Acceptance

For every asset update:

1. Load the full and light scenes from a newly promoted immutable release.
2. Confirm the scene marker and exactly the six expected gripper joints.
3. Close to `0.55` rad and verify all six joints converge with the table signs.
4. Open to `0.0` rad and verify all six return to zero.
5. Stop and restart simulation; readiness must degrade and recover without the
   bridge dying or requiring an Isaac restart.
6. Run the complete light-scene pick/place cycle. The object must lift, move,
   release, and fall at the drop pose; master-only convergence is failure.
7. Reset and repeat. Confirm there is one articulation root and one command
   writer after every reload.

The 2026-07-17 qualification passed close, open, readiness recovery, and the
full Composer pick/place cycle. The simulation was stopped after testing.

## Failure Diagnosis

- **Master moves, followers do not:** wrong/legacy gripper payload or missing
  PhysX mimic APIs. Do not command all joints as a workaround.
- **Followers diverge or oscillate:** competing controller/raw property writer,
  duplicate articulation root, or an unqualified force override.
- **Scene crashes during hot load:** verify the top robot layer retained the
  qualified arm graph and `root_joint` transform properties. Restore the thin
  sublayer over the known-good binary; do not hand-rebuild or copy that graph.
- **Fresh workstation cannot load the gripper:** verify access to the exact
  NVIDIA 5.1 URL (or populate the Isaac resolver cache), then reload and confirm
  the six-joint readiness contract.
- **Transient controller initialization warning after reset:** acceptable only
  if the graph immediately reinitializes, readiness returns, and repeated
  commands pass. Persistent failure is not qualified.

## Change Rules

- Preserve a single drive joint and single command writer.
- Never add scene-local follower drives to compensate for a broken asset.
- Never depend on browser/Composer polling to advance gripper state.
- Change the versioned upstream URL only in the wrapper and repeat full
  qualification; never copy/edit vendor meshes into the scene layers.
- Any topology, force, graph, or attachment change requires parser validation
  plus full live requalification.
