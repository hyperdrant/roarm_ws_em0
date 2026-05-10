import cv2
import numpy as np


H_FILE = "/home/hyperdrant/roarm_ws_em0/src/roarm_xyz_control/tools/homography_pixel_to_robot.npy"


def pixel_to_robot(u, v, H):
    pt = np.array([[[u, v]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H)
    x = float(out[0, 0, 0])
    y = float(out[0, 0, 1])
    return x, y


def main():
    H = np.load(H_FILE)

    while True:
        text = input("Input pixel u v, example: 319 253, or q: ").strip()

        if text.lower() in ["q", "quit", "exit"]:
            break

        parts = text.split()
        if len(parts) != 2:
            print("Please input two numbers: u v")
            continue

        u, v = map(float, parts)
        x, y = pixel_to_robot(u, v, H)

        print(f"pixel=({u:.1f}, {v:.1f}) -> robot=({x:.1f}, {y:.1f}) mm")


if __name__ == "__main__":
    main()
