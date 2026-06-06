#!/usr/bin/env python3
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from gazebo_msgs.msg import LinkStates
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from maritime_scene_utils import dynamic_mode_enabled, load_scene, scene_cloud_points, validate_scene


def clamp_rate(rate_hz):
    return max(2.0, min(8.0, float(rate_hz)))


def build_cloud(points, frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return point_cloud2.create_cloud_xyz32(header, points)


def parse_args():
    parser = argparse.ArgumentParser(description="Publish Gazebo-gated maritime obstacle PointCloud2.")
    parser.add_argument("--scene-config", default=None)
    parser.add_argument("--dynamic-mode", default="auto")
    return parser.parse_known_args()[0]


class GazeboCloud:
    def __init__(self):
        args = parse_args()
        scene_path = rospy.get_param("~scene_config", args.scene_config)
        self.scene = load_scene(scene_path)
        errors = validate_scene(self.scene)
        if errors:
            raise RuntimeError("invalid maritime scene: %s" % "; ".join(errors))

        self.frame_id = rospy.get_param("~frame_id", self.scene.get("frame_id", "world"))
        self.topic = rospy.get_param("~topic", "/maritime/gazebo_obstacles_cloud")
        self.dynamic_mode = rospy.get_param("~dynamic_mode", args.dynamic_mode)
        self.include_dynamic = dynamic_mode_enabled(self.scene, self.dynamic_mode)
        self.link_states_topic = rospy.get_param("~link_states_topic", "/gazebo/link_states")
        self.required_link_substring = rospy.get_param("~required_link_substring", "base_link")
        self.require_link_states = bool(rospy.get_param("~require_link_states", True))
        self.rate_hz = clamp_rate(rospy.get_param("~rate_hz", self.scene.get("cloud_publish_rate_hz", 3.0)))
        self.points = scene_cloud_points(self.scene, include_dynamic=self.include_dynamic)
        self.has_required_link = not self.require_link_states
        self.link_count = 0
        self.publisher = rospy.Publisher(self.topic, PointCloud2, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber(self.link_states_topic, LinkStates, self.link_states_callback, queue_size=1)
        rospy.loginfo("gazebo_cloud pseudo sensor waiting on %s then publishing points to %s dynamic_mode=%s",
                      self.link_states_topic, self.topic, self.dynamic_mode)

    def link_states_callback(self, msg):
        self.link_count = len(msg.name)
        if not self.require_link_states:
            self.has_required_link = True
            return
        self.has_required_link = any(self.required_link_substring in name for name in msg.name)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                if self.has_required_link:
                    if self.include_dynamic:
                        self.points = scene_cloud_points(self.scene, dynamic_time_sec=rospy.Time.now().to_sec())
                    self.publisher.publish(build_cloud(self.points, self.frame_id))
                rate.sleep()
        except rospy.ROSInterruptException:
            pass


def main():
    rospy.init_node("maritime_gazebo_cloud")
    GazeboCloud().spin()


if __name__ == "__main__":
    main()
