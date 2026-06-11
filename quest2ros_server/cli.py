from __future__ import annotations

import argparse


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Dual-arm Quest2ROS motion server.")
    parser.add_argument("--ros", choices=("ros1", "ros2"), default="ros1", help="ROS runtime to use.")
    parser.add_argument("--node-name", default="quest_motion_server")
    parser.add_argument("--rate-hz", type=float, default=30.0, help="Snapshot loop frequency for JSON output.")
    parser.add_argument("--trigger-threshold", type=float, default=0.5)
    parser.add_argument("--trigger-field", default="press_index", help="OVR2ROSInputs field used as trigger value.")
    parser.add_argument("--print-json", action="store_true", help="Print latest dual-hand snapshot every tick.")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.ros == "ros1":
        from .ros1_node import run
    else:
        from .ros2_node import run

    run(
        node_name=args.node_name,
        rate_hz=args.rate_hz,
        trigger_threshold=args.trigger_threshold,
        trigger_field=args.trigger_field,
        print_json=args.print_json,
    )


if __name__ == "__main__":
    main()
