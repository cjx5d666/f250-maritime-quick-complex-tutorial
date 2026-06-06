#!/usr/bin/env python3
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from maritime_scene_utils import dynamic_mode_enabled, dynamic_obstacle_center, dynamic_obstacle_yaw
from maritime_scene_utils import load_scene, scene_dynamic_obstacles, scene_waypoints
from maritime_scene_utils import scene_box


def color(r, g, b, a):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def quat_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def pose_from_center(center, yaw=0.0):
    pose = Pose()
    pose.position = Point(x=center[0], y=center[1], z=center[2])
    pose.orientation = quat_from_yaw(yaw)
    return pose


def scale3(value):
    if isinstance(value, (list, tuple)):
        vals = [float(v) for v in value[:3]]
        if len(vals) == 1:
            return [vals[0], vals[0], vals[0]]
        if len(vals) == 2:
            return [vals[0], vals[1], 1.0]
        return vals
    v = float(value)
    return [v, v, v]


def dynamic_mesh_pose(obstacle, dynamic_time_sec):
    center = list(dynamic_obstacle_center(obstacle, dynamic_time_sec))
    yaw = dynamic_obstacle_yaw(obstacle, dynamic_time_sec)
    mesh_pose = list(obstacle.get("mesh_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    while len(mesh_pose) < 6:
        mesh_pose.append(0.0)
    dx, dy, dz = float(mesh_pose[0]), float(mesh_pose[1]), float(mesh_pose[2])
    c = math.cos(yaw)
    s = math.sin(yaw)
    center[0] += c * dx - s * dy
    center[1] += s * dx + c * dy
    center[2] += dz
    return pose_from_center(center, yaw + float(mesh_pose[5]))


def make_marker(frame_id, marker_id, name, marker_type, center, scale, marker_color, yaw=0.0):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "f250_maritime_stage1"
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose = pose_from_center(center, yaw)
    marker.scale = Vector3(x=scale[0], y=scale[1], z=scale[2])
    marker.color = marker_color
    marker.lifetime = rospy.Duration(0.0)
    marker.text = name
    return marker


def make_mesh_marker(frame_id, marker_id, obstacle, dynamic_time_sec):
    mesh_uri = obstacle.get("mesh_uri")
    if not mesh_uri:
        return None
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "f250_maritime_dynamic_mesh"
    marker.id = marker_id
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.pose = dynamic_mesh_pose(obstacle, dynamic_time_sec)
    sx, sy, sz = scale3(obstacle.get("mesh_scale", obstacle.get("scale", 1.0)))
    marker.scale = Vector3(x=sx, y=sy, z=sz)
    marker.color = color(1.0, 1.0, 1.0, 1.0)
    marker.lifetime = rospy.Duration(0.0)
    marker.mesh_resource = str(mesh_uri)
    marker.mesh_use_embedded_materials = True
    marker.text = obstacle.get("name", "dynamic_obstacle") + "_visual_mesh"
    return marker


def build_marker_array(scene, frame_id, dynamic_time_sec=0.0, include_dynamic=True):
    markers = []
    delete_all = Marker()
    delete_all.header.frame_id = frame_id
    delete_all.header.stamp = rospy.Time.now()
    delete_all.action = Marker.DELETEALL
    markers.append(delete_all)

    marker_id = 0
    deck = scene_box(scene, "deck", required=True)
    markers.append(make_marker(frame_id, marker_id, deck["name"], Marker.CUBE, deck["center"], deck["size"],
                               color(0.38, 0.38, 0.34, 0.9)))
    marker_id += 1

    for item in scene.get("ship_hull", []) or []:
        markers.append(make_marker(frame_id, marker_id, item["name"], Marker.CUBE, item["center"], item["size"],
                                   color(0.17, 0.29, 0.36, 0.65)))
        marker_id += 1

    takeoff = scene_box(scene, "takeoff_deck_zone", required=True)
    markers.append(make_marker(frame_id, marker_id, takeoff["name"], Marker.CUBE, takeoff["center"], takeoff["size"],
                               color(0.18, 0.48, 0.95, 0.9)))
    marker_id += 1

    landing = scene_box(scene, "landing_deck_zone", required=True)
    markers.append(make_marker(frame_id, marker_id, landing["name"], Marker.CUBE, landing["center"], landing["size"],
                               color(0.15, 0.85, 0.25, 0.9)))
    marker_id += 1

    for pier in scene.get("bridge_piers", []) or []:
        markers.append(make_marker(frame_id, marker_id, pier["name"], Marker.CYLINDER, pier["center"],
                                   [2.0 * pier["radius"], 2.0 * pier["radius"], pier["height"]],
                                   color(0.72, 0.72, 0.68, 0.92)))
        marker_id += 1

    for buoy in scene.get("buoys", []) or []:
        buoy_color = buoy.get("color", [1.0, 0.18, 0.08, 1.0])
        markers.append(make_marker(frame_id, marker_id, buoy["name"], Marker.CYLINDER, buoy["center"],
                                   [2.0 * buoy["radius"], 2.0 * buoy["radius"], buoy["height"]],
                                   color(*buoy_color)))
        marker_id += 1

    for dock in scene.get("docks", []) or []:
        dock_color = dock.get("color", [0.34, 0.28, 0.20, 0.9])
        markers.append(make_marker(frame_id, marker_id, dock["name"], Marker.CUBE, dock["center"], dock["size"],
                                   color(*dock_color)))
        marker_id += 1

    for box in scene.get("box_obstacles", []) or []:
        obj_color = color(0.9, 0.52, 0.14, 0.88) if "container" in box["name"] else color(0.82, 0.82, 0.75, 0.85)
        markers.append(make_marker(frame_id, marker_id, box["name"], Marker.CUBE, box["center"], box["size"], obj_color))
        marker_id += 1

    for box in scene.get("visual_boxes", []) or []:
        obj_color = box.get("color", [0.72, 0.74, 0.74, 0.9])
        markers.append(make_marker(frame_id, marker_id, box["name"], Marker.CUBE,
                                   box["center"], box["size"], color(*obj_color)))
        marker_id += 1

    if include_dynamic:
        for obstacle in scene_dynamic_obstacles(scene):
            obj_color = obstacle.get("color", [0.95, 0.32, 0.12, 0.92])
            if len(obj_color) >= 4:
                obj_color = list(obj_color)
                obj_color[3] = min(float(obj_color[3]), 0.35)
            shape = str(obstacle.get("shape", "box")).lower()
            marker_type = Marker.CUBE
            scale = obstacle.get("size", [1.0, 1.0, 1.0])
            if shape == "cylinder":
                marker_type = Marker.CYLINDER
                scale = [2.0 * obstacle["radius"], 2.0 * obstacle["radius"], obstacle["height"]]
            markers.append(make_marker(
                frame_id, marker_id, obstacle["name"], marker_type,
                dynamic_obstacle_center(obstacle, dynamic_time_sec), scale,
                color(*obj_color), dynamic_obstacle_yaw(obstacle, dynamic_time_sec)))
            marker_id += 1
            mesh_marker = make_mesh_marker(frame_id, marker_id, obstacle, dynamic_time_sec)
            if mesh_marker is not None:
                markers.append(mesh_marker)
                marker_id += 1

    path_marker = Marker()
    path_marker.header.frame_id = frame_id
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "f250_maritime_stage1"
    path_marker.id = marker_id
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.pose.orientation.w = 1.0
    path_marker.scale.x = 0.08
    path_marker.color = color(0.1, 0.85, 1.0, 0.95)
    waypoints = scene_waypoints(scene)
    path_marker.points = []
    for waypoint in waypoints:
        x, y, z = waypoint["position"]
        path_marker.points.append(Point(x=x, y=y, z=z))
    markers.append(path_marker)
    marker_id += 1

    for waypoint in waypoints:
        markers.append(make_marker(frame_id, marker_id, waypoint["name"], Marker.SPHERE,
                                   waypoint["position"], [0.35, 0.35, 0.35],
                                   color(0.05, 0.95, 0.9, 0.95)))
        marker_id += 1

    return MarkerArray(markers=markers)


def build_path(scene, frame_id):
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = rospy.Time.now()
    for waypoint in scene_waypoints(scene):
        pose = Pose()
        pose.position = Point(
            x=waypoint["position"][0],
            y=waypoint["position"][1],
            z=waypoint["position"][2],
        )
        pose.orientation = quat_from_yaw(waypoint["yaw"])
        stamped = PoseStamped()
        stamped.header = path.header
        stamped.pose = pose
        path.poses.append(stamped)
    return path


def main():
    rospy.init_node("maritime_scene_markers")
    scene = load_scene(rospy.get_param("~scene_config", None))
    frame_id = rospy.get_param("~frame_id", scene.get("frame_id", "world"))
    marker_topic = rospy.get_param("~marker_topic", "/maritime/scene_markers")
    path_topic = rospy.get_param("~path_topic", "/maritime/mission_path")
    dynamic_mode = rospy.get_param("~dynamic_mode", "auto")
    include_dynamic = dynamic_mode_enabled(scene, dynamic_mode)
    rate_hz = max(1.0, min(5.0, float(rospy.get_param("~rate_hz", scene.get("cloud_publish_rate_hz", 3.0)))))

    marker_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=1, latch=True)
    path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)
    rospy.loginfo("publishing maritime scene markers on %s and %s", marker_topic, path_topic)
    try:
        while not rospy.is_shutdown():
            marker_pub.publish(build_marker_array(
                scene, frame_id, rospy.Time.now().to_sec(), include_dynamic=include_dynamic))
            path_pub.publish(build_path(scene, frame_id))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
