"""
Generate an ArUco marker PNG for printing.

After printing, MEASURE the actual side length of the inner black square
(not including the white quiet zone) with a ruler — printer scaling
varies, and `--marker-size-m` to calibrate_handeye.py must be the
*physical* size, not the nominal target.
"""
import argparse

import cv2


ARUCO_DICTS = {
    "DICT_4X4_50":  cv2.aruco.DICT_4X4_50,
    "DICT_5X5_50":  cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_50":  cv2.aruco.DICT_6X6_50,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dict", choices=list(ARUCO_DICTS), default="DICT_5X5_50")
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--size-px", type=int, default=600,
                    help="Image edge in pixels. 600 px @ 300 DPI = 5.08 cm.")
    ap.add_argument("--border-px", type=int, default=60,
                    help="White quiet zone around the marker.")
    ap.add_argument("--out", default="aruco_marker.png")
    args = ap.parse_args()

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[args.dict])
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(aruco_dict, args.id, args.size_px)
    else:
        marker = cv2.aruco.drawMarker(aruco_dict, args.id, args.size_px)

    if args.border_px > 0:
        marker = cv2.copyMakeBorder(
            marker, args.border_px, args.border_px, args.border_px, args.border_px,
            cv2.BORDER_CONSTANT, value=255,
        )

    cv2.imwrite(args.out, marker)
    print(f"saved {args.out}: dict={args.dict} id={args.id} {args.size_px}x{args.size_px} px"
          f" + {args.border_px} px white border")
    print("\nNext steps:")
    print( "  1. Print at 100% scale (no fit-to-page).")
    print( "  2. Measure the printed black-square side length with a ruler.")
    print( "  3. Pass that measurement (in meters) to calibrate_handeye.py via --marker-size-m.")


if __name__ == "__main__":
    main()
