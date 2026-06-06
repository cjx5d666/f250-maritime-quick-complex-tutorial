#!/usr/bin/env python3
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from maritime_scene_utils import dynamic_mode_enabled, load_scene, scene_bounds
from maritime_scene_utils import scene_cloud_points, validate_scene


def clamp_rate(rate_hz):
    return max(2.0, min(5.0, float(rate_hz)))


def build_cloud(points, frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return point_cloud2.create_cloud_xyz32(header, points)


def parse_args():
    parser = argparse.ArgumentParser(description="Publish deterministic maritime obstacle PointCloud2.")
    parser.add_argument("--scene-config", default=None)
    parser.add_argument("--dynamic-mode", default="auto")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_known_args()[0]


def main():
    args = parse_args()
    scene_path = args.scene_config
    if args.dry_run:
        scene = load_scene(scene_path)
        errors = validate_scene(scene)
        points = scene_cloud_points(scene, include_dynamic=dynamic_mode_enabled(scene, args.dynamic_mode))
        print({
            "ok": not errors,
            "errors": errors,
            "points": len(points),
            "bounds": scene_bounds(points),
            "scene": scene.get("_scene_path"),
        })
        raise SystemExit(0 if not errors else 2)

    rospy.init_node("maritime_obstacle_cloud")
    scene_path = rospy.get_param("~scene_config", scene_path)
    topic = rospy.get_param("~topic", "/maritime/obstacles_cloud")
    scene = load_scene(scene_path)
    errors = validate_scene(scene)
    if errors:
        raise RuntimeError("invalid maritime scene: %s" % "; ".join(errors))

    frame_id = rospy.get_param("~frame_id", scene.get("frame_id", "world"))
    dynamic_mode = rospy.get_param("~dynamic_mode", args.dynamic_mode)
    include_dynamic = dynamic_mode_enabled(scene, dynamic_mode)
    rate_hz = clamp_rate(rospy.get_param("~rate_hz", scene.get("cloud_publish_rate_hz", 3.0)))
    points = scene_cloud_points(scene, include_dynamic=include_dynamic)
    publisher = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)
    rospy.loginfo("publishing deterministic maritime obstacle points on %s at %.1f Hz dynamic_mode=%s",
                  topic, rate_hz, dynamic_mode)

    try:
        while not rospy.is_shutdown():
            if include_dynamic:
                points = scene_cloud_points(scene, dynamic_time_sec=rospy.Time.now().to_sec())
            publisher.publish(build_cloud(points, frame_id))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
