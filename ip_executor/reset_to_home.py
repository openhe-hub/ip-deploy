"""Reset Franka EE to demo's home pose. Read from layout.json or hardcoded.

Run this between IP episodes so each run starts at the same place.
"""
from __future__ import annotations
import argparse
import time
import torch
from polymetis import RobotInterface, GripperInterface

# From recordings/tasks_2026-05-06_14-53-28/layout.json
HOME_Q_RAD = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
HOME_EE_POS = [0.31369882822036743, 0.000659744197037071, 0.5897191762924194]
HOME_EE_QUAT_XYZW = [0.9262571334838867, -0.37645044922828674,
                     -0.002783368807286024, -0.018029851838946342]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-host", default="localhost")
    ap.add_argument("--robot-port", type=int, default=50051)
    ap.add_argument("--gripper-host", default="localhost")
    ap.add_argument("--gripper-port", type=int, default=50052)
    ap.add_argument("--no-gripper", action="store_true")
    args = ap.parse_args()

    print("[reset] connecting", flush=True)
    r = RobotInterface(ip_address=args.robot_host, port=args.robot_port)
    try:
        r.terminate_current_policy()
        time.sleep(0.5)
    except Exception:
        pass

    print(f"[reset] go to joint home {HOME_Q_RAD}", flush=True)
    r.move_to_joint_positions(torch.tensor(HOME_Q_RAD), time_to_go=4.0)

    pos, quat = r.get_ee_pose()
    print(f"[reset] EE settled at pos={pos.cpu().numpy().round(3).tolist()}"
          f" quat={quat.cpu().numpy().round(3).tolist()}", flush=True)

    if not args.no_gripper:
        g = GripperInterface(ip_address=args.gripper_host, port=args.gripper_port)
        g.goto(width=0.08, speed=0.1, force=10)
        # Panda hand goto from full close takes ~1s; poll up to 3s for stable.
        for _ in range(15):
            time.sleep(0.2)
            gs = g.get_state()
            w = float(getattr(gs, "width", -1))
            if w > 0.06:
                break
        print(f"[reset] gripper width={w:.3f}", flush=True)


if __name__ == "__main__":
    main()
