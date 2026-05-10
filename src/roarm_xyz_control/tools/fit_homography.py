import cv2
import numpy as np


# =========================
# 1. 图像像素坐标 uv
# 上面是你纸上记录的 UV
# 格式: [u, v]
# =========================
image_points = np.array([
    [606,  83],
    [300,  88],
    [ 43,  45],

    [616, 230],
    [319, 253],
    [ 38, 241],

    [610, 437],
    [328, 407],
    [ 38, 394],
], dtype=np.float32)


# =========================
# 2. 机械臂坐标 xy
# 这里建议用 mm，不要用 m
# 你的纸上是 0.21, 0.15 这种形式
# 所以这里乘以 1000
# 格式: [x_mm, y_mm]
# =========================
robot_points = np.array([
    [210,  150],
    [300,    0],
    [330, -150],

    [290,  180],
    [340,   40],
    [400,  -90],

    [400,  230],
    [450,   80],
    [470,    0],
], dtype=np.float32)


def main():
    if len(image_points) != len(robot_points):
        raise RuntimeError("image_points 和 robot_points 数量不一致")

    if len(image_points) < 4:
        raise RuntimeError("Homography 至少需要 4 个点")

    H, mask = cv2.findHomography(image_points, robot_points, method=0)

    if H is None:
        raise RuntimeError("Homography 计算失败")

    print("Homography pixel -> robot:")
    print(H)

    # 保存 .npy 文件
    output_file = "homography_pixel_to_robot.npy"
    np.save(output_file, H)
    print(f"\nSaved to: {output_file}")

    # =========================
    # 误差检查
    # =========================
    projected = cv2.perspectiveTransform(
        image_points.reshape(-1, 1, 2),
        H
    ).reshape(-1, 2)

    errors = np.linalg.norm(projected - robot_points, axis=1)

    print("\nCalibration errors:")
    for i, e in enumerate(errors):
        u, v = image_points[i]
        x_true, y_true = robot_points[i]
        x_pred, y_pred = projected[i]

        print(
            f"Point {i + 1}: "
            f"uv=({u:.1f},{v:.1f}) "
            f"true=({x_true:.1f},{y_true:.1f}) "
            f"pred=({x_pred:.1f},{y_pred:.1f}) "
            f"error={e:.2f} mm"
        )

    print(f"\nMean error: {errors.mean():.2f} mm")
    print(f"Max error:  {errors.max():.2f} mm")


if __name__ == "__main__":
    main()
