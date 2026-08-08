"""Read-only verification of a saved tool/frame mapping against open RoboDK."""

import argparse
import json
import os
import sys
from types import SimpleNamespace


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from pose_transform import load_extrinsic_config, transform_pose_records
import robodk_bridge


def main(argv=None):
    parser = argparse.ArgumentParser(description="Verify RoboDK station mapping read-only")
    parser.add_argument("config")
    args = parser.parse_args(argv)
    config = load_extrinsic_config(args.config)
    dummy = [{"index": 1, "x": 0, "y": 0, "z": 0,
              "qw": 1, "qx": 0, "qy": 0, "qz": 0}]
    _commands, metadata = transform_pose_records(dummy, config)
    result = SimpleNamespace(diagnostics=metadata)
    api = robodk_bridge._import_api()
    rdk = api["Robolink"]()
    robot = robodk_bridge._require_item(
        rdk, config.robodk_robot_name, api["ITEM_TYPE_ROBOT"], "robot")
    frame = robodk_bridge._require_item(
        rdk, config.robodk_frame_name, api["ITEM_TYPE_FRAME"], "frame")
    tool = robodk_bridge._require_item(
        rdk, config.robodk_tool_name, api["ITEM_TYPE_TOOL"], "tool")
    verification = robodk_bridge._verify_tool_mapping(
        rdk, robot, frame, tool, result,
        config.robodk_robot_name, config.robodk_frame_name, config.robodk_tool_name)
    print(json.dumps(verification, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
