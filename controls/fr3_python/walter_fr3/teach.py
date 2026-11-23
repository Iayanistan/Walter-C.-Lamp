"""Print the FR3's current joint angles as a stations YAML line (radians).

    python -m walter_fr3.teach --ip 192.168.58.2
Jog the arm to a pose with the WebApp or pendant, run this, paste the line under the station.
"""

import argparse

from .arm import FairinoArm


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ip", default="192.168.58.2")
    ap.add_argument("--pose", default="above", help="pose name to print (above, grasp, under, pour, start)")
    a = ap.parse_args()
    arm = FairinoArm(a.ip, enable=False)
    print(f"      {a.pose}: [{', '.join(f'{v:.4f}' for v in arm.joints())}]")
    arm.close()


if __name__ == "__main__":
    main()
