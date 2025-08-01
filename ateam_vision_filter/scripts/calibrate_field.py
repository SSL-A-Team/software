#! /usr/bin/python3

from dataclasses import dataclass
from typing import List
import sys
import rclpy
from rclpy.node import Node
import rosbag2_py
from ssl_league_msgs.msg import VisionWrapper
from ateam_msgs.msg import Overlay, OverlayArray, FieldInfo, RobotState
from geometry_msgs.msg import Point, Vector3
from rclpy.serialization import deserialize_message
import math
import numpy
import time


@dataclass
class FieldPoint:
    name: str
    x: float
    y: float

@dataclass
class Robot:
    id: int
    xs: List[float]
    ys: List[float]

@dataclass
class Ball:
    xs: List[float]
    ys: List[float]


def pointDist(x1, y1, x2, y2):
    return math.hypot((x1-x2), (y1-y2))


def getClosestPoint(query_x: float, query_y: float, field_points: List[FieldPoint]):
    return min(field_points, key=lambda fp: pointDist(query_x, query_y, fp.x, fp.y))


def teamTopicName(color, id):
    return f"/{color}_team/robot{id}"


def main():
    rclpy.init()

    color = sys.argv[1]
    bag_uri = sys.argv[2]

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_uri,
        storage_id='mcap'
    )
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    node = Node("calibrate_field")
    overlay_publisher = node.create_publisher(OverlayArray, "/overlays", 1)
    field_publisher = node.create_publisher(FieldInfo, "/field", 1)
    blue_robots_pub = {teamTopicName("blue", i): node.create_publisher(RobotState, teamTopicName("blue", i), 1) for i in range(16)}
    yellow_robots_pub = {teamTopicName("yellow", i): node.create_publisher(RobotState, teamTopicName("yellow", i), 1) for i in range(16)}

    robots = [Robot(id,[],[]) for id in range(16)]
    balls = []

    field_geometry = None

    while reader.has_next():
        topic, msg_ser, _ = reader.read_next()
        if topic == '/field':
            field_publisher.publish(msg_ser)
        if topic.startswith('/blue_team'):
            blue_robots_pub[topic].publish(msg_ser)
        if topic.startswith('/yellow_team'):
            yellow_robots_pub[topic].publish(msg_ser)
        if topic == '/vision_messages':
            msg = deserialize_message(msg_ser, VisionWrapper)
            if len(msg.detection) > 0:
                detections = msg.detection[0]
                for i in range(len(detections.balls)):
                    if i >= len(balls):
                        balls.append(Ball([],[]))
                    ball = balls[i]
                    detect_ball = detections.balls[i]
                    ball.xs.append(detect_ball.pos.x)
                    ball.ys.append(detect_ball.pos.y)
                robot_detections = detections.robots_blue if color == 'blue' else detections.robots_yello
                for detect_robot in robot_detections:
                    robot = robots[detect_robot.robot_id]
                    robot.xs.append(detect_robot.pose.position.x)
                    robot.ys.append(detect_robot.pose.position.y)
            if len(msg.geometry) > 0:
                field_geometry = msg.geometry[0].field
    
    if field_geometry is None:
        print(f"Did not find field geoemtry packet in bag")
        return 1
    
    half_field_len = field_geometry.field_length / 2
    half_field_wid = field_geometry.field_width / 2
    def_area_front_abs = half_field_len - field_geometry.penalty_area_depth

    field_points = [
        FieldPoint("Center", 0.0, 0.0),
        FieldPoint("NegNegCorner", -half_field_len, -half_field_wid),
        FieldPoint("NegPosCorner", -half_field_len, half_field_wid),
        FieldPoint("PosPosCorner", half_field_len, half_field_wid),
        FieldPoint("PosNegCorner", half_field_len, -half_field_wid),
        FieldPoint("NegSideCenter", 0.0, -half_field_wid),
        FieldPoint("PosSideCenter", 0.0, half_field_wid),
        FieldPoint("NegGoalCenter", -half_field_len, 0.0),
        FieldPoint("PosGoalCenter", half_field_len, 0.0),
        FieldPoint("NegDefAreaFrontCenter", -def_area_front_abs, 0.0),
        FieldPoint("PosDefAreaFrontCenter", def_area_front_abs, 0.0),
        FieldPoint("NegDefAreaNegCorner", -def_area_front_abs, -field_geometry.penalty_area_width / 2),
        FieldPoint("NegDefAreaPosCorner", -def_area_front_abs, field_geometry.penalty_area_width / 2),
        FieldPoint("PosDefAreaNegCorner", def_area_front_abs, -field_geometry.penalty_area_width / 2),
        FieldPoint("PosDefAreaPosCorner", def_area_front_abs, field_geometry.penalty_area_width / 2),
    ]

    overlays = OverlayArray()

    viz_robot_points = []
    viz__robot_error_vectors = []
    x_errors = []
    y_errors = []
    for i in range(len(robots)):
        robot = robots[i]
        num_detections = len(robot.xs)
        if num_detections > 0:
            print(f"Robot {i}")
            print(f"\tFound {num_detections} detections.")
            x = numpy.average(robot.xs) - 0.025
            y = numpy.average(robot.ys) - 0.025
            closest_field_point = getClosestPoint(x, y, field_points)
            print(f"\tClosest field point is {closest_field_point.name}")
            error_mag = pointDist(x, y, closest_field_point.x, closest_field_point.y)
            error_x = closest_field_point.x - x
            error_y = closest_field_point.y - y
            x_errors.append(error_x)
            y_errors.append(error_y)
            print(f"\tError: {error_mag}  (x: {error_x}, y: {error_y})")
            viz_robot_points.append(Point(x=x, y=y))
            viz__robot_error_vectors.append(Vector3(x=error_x * 100, y=error_y * 100, z=0.0))
    print(f"Average X error: {numpy.average(x_errors)}")
    print(f"Average Y error: {numpy.average(y_errors)}")
           
    overlays.overlays.append(Overlay(
        ns="calibrate_field",
        name=f"robot_errors",
        visible=True,
        type=9,
        command=0,
        points=viz_robot_points,
        scales=viz__robot_error_vectors,
        stroke_color="#0044FFFF",
        stroke_width=5,
        lifetime=0,
        depth=1
    ))

    viz_ball_points = []
    viz__ball_error_vectors = []
    for i in range(len(balls)):
        ball = balls[i]
        num_detections = len(ball.xs)
        print(f"Ball {i}")
        print(f"\tFound {num_detections} detections.")
        x = numpy.average(ball.xs)
        y = numpy.average(ball.ys)
        closest_field_point = getClosestPoint(x, y, field_points)
        print(f"\tClosest field point is {closest_field_point.name}")
        error_mag = pointDist(x, y, closest_field_point.x, closest_field_point.y)
        error_x = closest_field_point.x - x
        error_y = closest_field_point.y - y
        print(f"\tError: {error_mag}  (x: {error_x}, y: {error_y})")
        viz_ball_points.append(Point(x=x, y=y))
        viz__ball_error_vectors.append(Vector3(x=error_x * 100, y=error_y * 100, z=0.0))
            
    overlays.overlays.append(Overlay(
        ns="calibrate_field",
        name=f"ball_errors",
        visible=True,
        type=9,
        command=0,
        points=viz_ball_points,
        scales=viz__ball_error_vectors,
        stroke_color="#FFA600FF",
        stroke_width=5,
        lifetime=0,
        depth=1
    ))

    print(f"Found {len(balls)} balls on the field.")

    overlay_publisher.publish(overlays)
    time.sleep(0.2)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
