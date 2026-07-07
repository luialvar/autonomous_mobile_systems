#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial import KDTree


@dataclass
class ICPResult:
    transform: np.ndarray
    aligned: np.ndarray
    iterations: int
    rmse: float
    correspondences: int
    converged: bool


def make_transform(x: float, y: float, yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array(
        [
            [c, -s, x],
            [s, c, y],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points.reshape(0, 2)

    homogeneous = np.column_stack((points[:, 0], points[:, 1], np.ones(len(points))))
    transformed = (transform @ homogeneous.T).T
    return transformed[:, :2]


def best_fit_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)

    source_centered = source - source_centroid
    target_centered = target - target_centroid

    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T

    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    translation = target_centroid - rotation @ source_centroid

    transform = np.eye(3)
    transform[:2, :2] = rotation
    transform[:2, 2] = translation
    return transform


def icp_point_to_point(
    source: np.ndarray,
    target: np.ndarray,
    *,
    initial_transform: Optional[np.ndarray] = None,
    max_iterations: int = 60,
    tolerance: float = 1e-5,
    max_correspondence_distance: float = 0.5,
    min_correspondences: int = 12,
) -> ICPResult:
    source = np.asarray(source, dtype=float).reshape(-1, 2)
    target = np.asarray(target, dtype=float).reshape(-1, 2)

    if len(source) < min_correspondences or len(target) < min_correspondences:
        return ICPResult(np.eye(3), source.copy(), 0, float("inf"), 0, False)

    total_transform = np.eye(3) if initial_transform is None else initial_transform.copy()
    aligned = transform_points(total_transform, source)
    tree = KDTree(target)
    last_rmse = float("inf")
    rmse = float("inf")
    correspondences = 0
    converged = False
    performed_iterations = 0

    for iteration in range(1, max_iterations + 1):
        performed_iterations = iteration
        distances, indices = tree.query(aligned)

        valid = np.isfinite(distances)
        if max_correspondence_distance > 0.0:
            valid &= distances <= max_correspondence_distance

        correspondences = int(np.count_nonzero(valid))
        if correspondences < min_correspondences:
            performed_iterations = iteration - 1
            break

        matched_source = aligned[valid]
        matched_target = target[indices[valid]]

        delta = best_fit_transform(matched_source, matched_target)
        aligned = transform_points(delta, aligned)
        total_transform = delta @ total_transform

        new_distances, _ = tree.query(aligned)
        new_distances = new_distances[valid]
        rmse = float(np.sqrt(np.mean(new_distances * new_distances)))

        if abs(last_rmse - rmse) < tolerance:
            converged = True
            return ICPResult(total_transform, aligned, iteration, rmse, correspondences, converged)

        last_rmse = rmse

    return ICPResult(total_transform, aligned, performed_iterations, rmse, correspondences, converged)


def quaternion_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def scan_to_local_points(scan_msg, *, beam_stride: int = 1) -> np.ndarray:
    points = []
    angle = scan_msg.angle_min

    for index, distance in enumerate(scan_msg.ranges):
        if index % beam_stride != 0:
            angle += scan_msg.angle_increment
            continue

        valid = (
            math.isfinite(distance)
            and scan_msg.range_min < distance < scan_msg.range_max
        )
        if valid:
            points.append((distance * math.cos(angle), distance * math.sin(angle)))

        angle += scan_msg.angle_increment

    return np.asarray(points, dtype=float).reshape(-1, 2)


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if len(points) == 0 or voxel_size <= 0.0:
        return points

    voxel_keys = np.floor(points / voxel_size).astype(np.int64)
    _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
    return points[np.sort(unique_indices)]


class ICPNode:
    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2

        class _Node(Node):
            pass

        self.node = _Node("icp_scan_matching")

        self.max_iterations = self.node.declare_parameter("max_iterations", 60).value
        self.tolerance = self.node.declare_parameter("tolerance", 1e-5).value
        self.max_correspondence_distance = self.node.declare_parameter(
            "max_correspondence_distance", 0.5
        ).value
        self.min_correspondences = self.node.declare_parameter("min_correspondences", 12).value
        self.match_every_n_scans = max(
            1,
            self.node.declare_parameter("match_every_n_scans", 1).value,
        )
        self.publish_every_n_scans = max(
            1,
            self.node.declare_parameter("publish_every_n_scans", 5).value,
        )
        self.beam_stride = max(1, self.node.declare_parameter("beam_stride", 2).value)
        self.voxel_size = self.node.declare_parameter("voxel_size", 0.03).value
        self.max_meta_points = self.node.declare_parameter("max_meta_points", 200000).value
        self.frame_id = self.node.declare_parameter("frame_id", "map").value

        self.latest_odom = None
        self.initial_odom_transform = None
        self.previous_scan = None
        self.meta_scan = np.empty((0, 2), dtype=float)
        self.received_scans = 0
        self.accepted_scans = 0

        self.scan_sub = self.node.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.node.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10,
        )
        self.meta_scan_pub = self.node.create_publisher(PointCloud2, "meta_scan", 10)

        self.node.get_logger().info("ICP scan matching node started")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        current = make_transform(position.x, position.y, yaw)

        if self.initial_odom_transform is None:
            self.initial_odom_transform = np.linalg.inv(current)

        self.latest_odom = self.initial_odom_transform @ current

    def scan_callback(self, msg):
        if self.latest_odom is None:
            self.node.get_logger().warn("Skipping scan until odometry is available", throttle_duration_sec=2.0)
            return

        self.received_scans += 1
        if self.received_scans % self.match_every_n_scans != 0:
            return

        local_points = scan_to_local_points(msg, beam_stride=self.beam_stride)
        if len(local_points) < self.min_correspondences:
            self.node.get_logger().warn("Skipping scan with too few valid points", throttle_duration_sec=2.0)
            return

        odom_guess = transform_points(self.latest_odom, local_points)

        if self.previous_scan is None:
            aligned = odom_guess
            result = None
        else:
            result = icp_point_to_point(
                odom_guess,
                self.previous_scan,
                max_iterations=self.max_iterations,
                tolerance=self.tolerance,
                max_correspondence_distance=self.max_correspondence_distance,
                min_correspondences=self.min_correspondences,
            )
            aligned = result.aligned

        self.previous_scan = aligned
        self.append_to_meta_scan(aligned)
        self.accepted_scans += 1

        if result is not None:
            self.node.get_logger().info(
                "ICP scan %d: rmse=%.4f correspondences=%d iterations=%d converged=%s"
                % (
                    self.accepted_scans,
                    result.rmse,
                    result.correspondences,
                    result.iterations,
                    result.converged,
                )
            )

        if self.accepted_scans % self.publish_every_n_scans == 0:
            self.publish_meta_scan()

    def append_to_meta_scan(self, scan_points: np.ndarray) -> None:
        self.meta_scan = np.vstack((self.meta_scan, scan_points))
        self.meta_scan = voxel_downsample(self.meta_scan, self.voxel_size)

        if len(self.meta_scan) > self.max_meta_points:
            step = math.ceil(len(self.meta_scan) / self.max_meta_points)
            self.meta_scan = self.meta_scan[::step]

    def publish_meta_scan(self) -> None:
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header

        if len(self.meta_scan) == 0:
            return

        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        xyz = np.column_stack((self.meta_scan, np.zeros(len(self.meta_scan))))
        msg = point_cloud2.create_cloud_xyz32(header, xyz.tolist())
        self.meta_scan_pub.publish(msg)


def run_node(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node_wrapper = ICPNode()
    try:
        rclpy.spin(node_wrapper.node)
    finally:
        node_wrapper.node.destroy_node()
        rclpy.shutdown()


def demo_points() -> np.ndarray:
    x = np.linspace(-2.0, 2.0, 70)
    top = np.column_stack((x, np.full_like(x, 1.0)))
    bottom = np.column_stack((x, np.full_like(x, -1.0)))
    left = np.column_stack((np.full_like(x, -2.0), np.linspace(-1.0, 1.0, 70)))
    right = np.column_stack((np.full_like(x, 2.0), np.linspace(-1.0, 1.0, 70)))
    diagonal = np.column_stack((np.linspace(-1.5, 1.5, 60), np.linspace(-0.7, 0.7, 60)))
    return np.vstack((top, bottom, left, right, diagonal))


def run_demo(output: Optional[Path] = None) -> ICPResult:
    rng = np.random.default_rng(9)
    target = demo_points()
    true_transform = make_transform(0.65, -0.35, math.radians(18.0))
    source = transform_points(true_transform, target)
    source += rng.normal(0.0, 0.015, source.shape)

    result = icp_point_to_point(
        source,
        target,
        max_iterations=80,
        tolerance=1e-7,
        max_correspondence_distance=1.0,
        min_correspondences=30,
    )

    print("Synthetic ICP demo")
    print(f"iterations: {result.iterations}")
    print(f"rmse: {result.rmse:.6f}")
    print(f"correspondences: {result.correspondences}")
    print(f"converged: {result.converged}")
    print("estimated transform:")
    print(np.array2string(result.transform, precision=4, suppress_small=True))

    if output is not None:
        import matplotlib.pyplot as plt

        output.parent.mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(7, 5))
        ax.scatter(target[:, 0], target[:, 1], s=10, label="target scan")
        ax.scatter(source[:, 0], source[:, 1], s=10, label="source before ICP")
        ax.scatter(result.aligned[:, 0], result.aligned[:, 1], s=10, label="source after ICP")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, linewidth=0.4)
        ax.legend(loc="best")
        ax.set_title("2D ICP synthetic verification")
        fig.tight_layout()
        fig.savefig(output, dpi=160)
        plt.close(fig)

    return result


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="2D ICP scan matching node")
    parser.add_argument("--demo", action="store_true", help="run a synthetic ICP verification")
    parser.add_argument("--demo-output", type=Path, default=None, help="optional output image for the demo")
    parsed, ros_args = parser.parse_known_args(args)

    if parsed.demo:
        run_demo(parsed.demo_output)
        return

    run_node(ros_args)


if __name__ == "__main__":
    main()
