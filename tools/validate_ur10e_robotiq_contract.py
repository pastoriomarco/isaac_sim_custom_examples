#!/usr/bin/env python3
"""Validate the local UR10e/Robotiq scene-composition authority contract."""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Sdf, Usd


ROBOT_INSTANCE = "/World/ur10e_robotiq2f_140_ROS/ur10e_robotiq2f_140"
ROBOT_LAYER_ROOT = "/ur10e_robotiq2f_140_ROS"
ROBOT_LAYER_INSTANCE = f"{ROBOT_LAYER_ROOT}/ur10e_robotiq2f_140"
CONTROLLER = "/Root/ActionGraph/articulation_controller"
GRAPH = "/Root/ActionGraph"
COMMAND_SUBSCRIBER = f"{GRAPH}/ros2_subscribe_joint_state"
PLAYBACK_TRIGGER = f"{GRAPH}/on_playback_tick"
SCENE_CONTROLLER = "/World/robotiq_action_graph_01/ActionGraph/articulation_controller"
CONTRACT_MARKER = "/World/ManyForgeContracts/RobotiqSingleAuthorityV1"
OFFICIAL_GRIPPER_ASSET = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/"
    "Isaac/5.1/Isaac/Samples/Rigging/Manipulator/import_manipulator/"
    "robotiq_2f_140/robotiq_2f_140.usd"
)
EXPECTED_ROBOT_TARGET = Sdf.Path(f"{ROBOT_INSTANCE}/root_joint")
EXPECTED_EXEC_SOURCE = Sdf.Path(
    f"{COMMAND_SUBSCRIBER}.outputs:execOut"
)
EXPECTED_POLL_SOURCE = Sdf.Path(f"{PLAYBACK_TRIGGER}.outputs:tick")
STALE_ROBOT_PRIMS = (
    f"{ROBOT_INSTANCE}/Robotiq_2F_140_config",
    f"{ROBOT_INSTANCE}/gripper",
    f"{ROBOT_INSTANCE}/ActionGraph",
)
STALE_ROBOT_LAYER_PRIMS = (
    f"{ROBOT_LAYER_INSTANCE}/Robotiq_2F_140_config",
    f"{ROBOT_LAYER_INSTANCE}/gripper",
)


def _open_layer(path: Path) -> Sdf.Layer:
    layer = Sdf.Layer.FindOrOpen(str(path))
    if layer is None:
        raise AssertionError(f"cannot open USD layer: {path}")
    return layer


def _items(list_op) -> list:
    if list_op.isExplicit:
        return list(list_op.explicitItems)
    return [
        *list(list_op.prependedItems),
        *list(list_op.addedItems),
        *list(list_op.appendedItems),
    ]


def _walk(prim: Sdf.PrimSpec):
    yield prim
    for child in prim.nameChildren:
        yield from _walk(child)


def _validate_graph(root: Path) -> None:
    layer = _open_layer(
        root / "Scenario/IsaacManipulator/UniversalRobots/robotiq_action_graph.usda"
    )
    graph = layer.GetPrimAtPath(GRAPH)
    assert graph is not None, "gripper command graph is missing"
    assert graph.attributes["pipelineStage"].default == "pipelineStageSimulation"

    trigger = layer.GetPrimAtPath(PLAYBACK_TRIGGER)
    assert trigger is not None, "playback trigger is missing"
    assert trigger.attributes["node:type"].default == "omni.graph.action.OnPlaybackTick"

    subscriber = layer.GetPrimAtPath(COMMAND_SUBSCRIBER)
    assert subscriber is not None, "gripper command subscriber is missing"
    assert _items(subscriber.attributes["inputs:execIn"].connectionPathList) == [
        EXPECTED_POLL_SOURCE
    ]

    controller = layer.GetPrimAtPath(CONTROLLER)
    assert controller is not None, "gripper articulation controller is missing"
    assert controller.attributes["inputs:jointNames"].default == ["finger_joint"]
    assert _items(
        controller.attributes["inputs:execIn"].connectionPathList
    ) == [EXPECTED_EXEC_SOURCE]
    assert "inputs:targetPrim" not in controller.relationships, (
        "reusable payload must not target a prim outside its namespace"
    )
    assert "inputs:effortCommand" not in controller.attributes, (
        "position-commanded gripper must not receive a competing effort command"
    )

    controllers = []
    raw_writers = []
    for root_prim in layer.rootPrims:
        for prim in _walk(root_prim):
            node_type = prim.attributes.get("node:type")
            node_type = str(node_type.default) if node_type is not None else ""
            if node_type.endswith("IsaacArticulationController"):
                controllers.append(str(prim.path))
            if "WritePrimAttribute" in node_type:
                raw_writers.append(str(prim.path))
    assert controllers == [CONTROLLER], f"unexpected controllers: {controllers}"
    assert not raw_writers, f"raw drive writers are forbidden: {raw_writers}"


def _validate_gripper_composition(root: Path) -> None:
    layer = _open_layer(
        root
        / "Robots/UniversalRobots/ur10e/configuration/"
        "ur10e_Gripper_2F_140_manyforge.usda"
    )
    ee_link = layer.GetPrimAtPath("/ur10e/ee_link")
    payloads = [item.assetPath for item in _items(ee_link.payloadList)]
    assert payloads == [OFFICIAL_GRIPPER_ASSET], payloads
    assert layer.GetPrimAtPath("/ur10e/ee_link/root_joint").active is False

    max_force = layer.GetPropertyAtPath(
        "/ur10e/ee_link/joints/finger_joint.drive:angular:physics:maxForce"
    )
    assert max_force is not None and max_force.default == 200.0

    attachment = layer.GetPropertyAtPath("/ur10e/joints/ee_joint.physics:body1")
    assert _items(attachment.targetPathList) == [
        Sdf.Path("/ur10e/ee_link/robotiq_arg2f_base_link")
    ]


def _validate_ur_overlay(root: Path) -> None:
    ur_root = root / "Robots/UniversalRobots/ur10e"
    layer = _open_layer(ur_root / "ur10e_manyforge.usda")
    assert layer.subLayerPaths == ["./ur10e.usda"]
    prim = layer.GetPrimAtPath("/ur10e")
    assert prim is not None
    variant = prim.variantSets["Gripper"].variants["Robotiq_2f_140"].primSpec
    assert [item.assetPath for item in variant.payloadList.prependedItems] == [
        "./configuration/ur10e_Gripper_2F_140_manyforge.usda"
    ]
    assert [item.assetPath for item in variant.payloadList.deletedItems] == [
        "./configuration/ur10e_Gripper_2F_140.usd"
    ]
    assert "manyforge" not in (ur_root / "ur10e.usda").read_text(
        encoding="utf-8"
    ).lower(), "shared UR asset must remain independent of ManyForge overlays"


def _validate_robot_layer(root: Path) -> None:
    layer = _open_layer(root / "Robots/ur10e_robotiq2f-140_ROS_manyforge.usda")
    assert layer.subLayerPaths == ["./ur10e_robotiq2f-140_ROS.usd"]
    robot = layer.GetPrimAtPath(ROBOT_LAYER_INSTANCE)
    assert robot is not None, "robot payload override is missing"
    assert [item.assetPath for item in robot.payloadList.prependedItems] == [
        "./UniversalRobots/ur10e/ur10e_manyforge.usda"
    ]
    assert [item.assetPath for item in robot.payloadList.deletedItems] == [
        "./UniversalRobots/ur10e/ur10e.usd"
    ]

    for stale_path in STALE_ROBOT_LAYER_PRIMS:
        stale = layer.GetPrimAtPath(stale_path)
        assert stale is not None and stale.active is False, stale_path

    original = _open_layer(root / "Robots/ur10e_robotiq2f-140_ROS.usd")
    original_robot = original.GetPrimAtPath(ROBOT_LAYER_INSTANCE)
    assert original_robot is not None
    assert original_robot.variantSelections["Gripper"] == "Robotiq_2f_140"

    graph = original.GetPrimAtPath(f"{ROBOT_LAYER_ROOT}/ur_action_graph")
    assert graph is not None, "qualified arm command graph is missing"
    assert graph.attributes["pipelineStage"].default == "pipelineStageSimulation"
    trigger = original.GetPrimAtPath(f"{graph.path}/on_playback_tick")
    assert trigger is not None
    assert trigger.attributes["node:type"].default == (
        "omni.graph.action.OnPlaybackTick"
    )

    root_joint = original.GetPrimAtPath(f"{ROBOT_LAYER_INSTANCE}/root_joint")
    assert root_joint is not None, "articulation root override is missing"
    assert root_joint.attributes["xformOp:scale"].default == (1.0, 1.0, 1.0)
    assert root_joint.attributes["xformOpOrder"].default == [
        "xformOp:translate",
        "xformOp:orient",
        "xformOp:scale",
    ]

    stage = Usd.Stage.Open(str(layer.realPath), load=Usd.Stage.LoadNone)
    assert stage is not None
    assert stage.GetPrimAtPath(f"{ROBOT_LAYER_ROOT}/ur_action_graph").IsValid()
    for stale_path in STALE_ROBOT_LAYER_PRIMS:
        stale = stage.GetPrimAtPath(stale_path)
        assert stale.IsValid() and not stale.IsActive(), stale_path


def _validate_scene(root: Path, scene_name: str) -> None:
    layer = _open_layer(
        root / "Scenario/IsaacManipulator/UniversalRobots" / scene_name
    )
    robot = layer.GetPrimAtPath("/World/ur10e_robotiq2f_140_ROS")
    robot_payloads = [item.assetPath for item in _items(robot.payloadList)]
    assert robot_payloads == [
        "../../../Robots/ur10e_robotiq2f-140_ROS_manyforge.usda"
    ]

    marker = layer.GetPrimAtPath(CONTRACT_MARKER)
    assert marker is not None, "live-queryable gripper contract marker is missing"
    assert marker.attributes["contract"].default == (
        "one_master_drive_six_joint_settled_completion_v1"
    )

    graph = layer.GetPrimAtPath("/World/robotiq_action_graph_01")
    graph_payloads = [item.assetPath for item in _items(graph.payloadList)]
    assert graph_payloads == ["./robotiq_action_graph.usda"]

    for stale_path in STALE_ROBOT_PRIMS:
        assert layer.GetPrimAtPath(stale_path) is None, stale_path
    ee_link = layer.GetPrimAtPath(f"{ROBOT_INSTANCE}/ee_link")
    assert ee_link is not None and not list(ee_link.nameChildren), (
        "scene must not override gripper-model children"
    )

    controller = layer.GetPrimAtPath(SCENE_CONTROLLER)
    assert controller is not None, "scene-local controller binding is missing"
    assert {prop.name for prop in controller.properties} == {"inputs:targetPrim"}
    target = controller.relationships["inputs:targetPrim"]
    assert _items(target.targetPathList) == [EXPECTED_ROBOT_TARGET]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--asset-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    args = parser.parse_args()
    root = args.asset_root.resolve()

    _validate_graph(root)
    _validate_gripper_composition(root)
    _validate_ur_overlay(root)
    _validate_robot_layer(root)
    _validate_scene(root, "isaac_manipulator_scene.usda")
    _validate_scene(root, "isaac_manipulator_scene_light.usda")
    print("UR10e/Robotiq USD authority contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
