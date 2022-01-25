#!/usr/bin/env python
"""***********************************************************************************
     C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
     Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
     Copyright (C) 2022 Bai Li
     Users are suggested to cite the following article when they use the source codes.
     Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
     Frenet Frame: A Cartesian-based Trajectory Planning Method",
     IEEE Transactions on Intelligent Transportation Systems, 2022.
***********************************************************************************"""
import bisect
import sys
import rospy
import numpy as np
import pickle
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cartesian_planner.msg import CenterLine, CenterLinePoint, Obstacles, DynamicObstacles, DynamicObstacle, \
    DynamicTrajectoryPoint
from geometry_msgs.msg import Polygon, Point32


def generate_center_line(config, start_x=0.0, start_y=0.0, start_yaw=0.0, resolution=0.1, left_bound=2.5,
                         right_bound=6.0):
    x = start_x
    y = start_y
    yaw = start_yaw

    incremental_s = 0.0
    center_line = CenterLine()
    center_line.points = [
        CenterLinePoint(s=0.0, x=x, y=y, theta=yaw, kappa=0.0, left_bound=left_bound, right_bound=right_bound)]

    for seg in config:
        if isinstance(seg, list):
            [degree, radius] = seg
            angle = np.deg2rad(degree)
            arc_direction = -1 if angle < 0 else 1
            arc_length = angle * radius
            kappa = 1 / radius * arc_direction
            start_angle = yaw - np.pi / 2 * arc_direction
            end_angle = start_angle + angle

            center_yaw = yaw + np.pi / 2 * arc_direction
            xc = x + radius * np.cos(center_yaw)
            yc = y + radius * np.sin(center_yaw)

            point_count = np.floor(np.abs(arc_length) / resolution)
            angles = np.linspace(start_angle, end_angle, point_count)

            yaw_inc = angle / point_count

            for ang in angles:
                x = xc + radius * np.cos(ang)
                y = yc + radius * np.sin(ang)
                incremental_s += resolution
                yaw += yaw_inc

                center_line.points.append(
                    CenterLinePoint(s=incremental_s, x=x, y=y, theta=yaw, kappa=kappa, left_bound=left_bound,
                                    right_bound=right_bound))
        else:
            for i in range(int(seg / resolution)):
                x += resolution * np.cos(yaw)
                y += resolution * np.sin(yaw)
                incremental_s += resolution
                center_line.points.append(
                    CenterLinePoint(s=incremental_s, x=x, y=y, theta=yaw, kappa=0, left_bound=left_bound,
                                    right_bound=right_bound))

    if len(center_line.points) > 1:
        center_line.points[0].kappa = center_line.points[1].kappa
    return center_line


def convert_frenet_to_cartesian(ref, laterals):
    xs = ref[:, 1] - laterals * np.sin(ref[:, 3])
    ys = ref[:, 2] + laterals * np.cos(ref[:, 3])
    return xs, ys


def transform_footprint(x, y, theta, footprint):
    points = [
        [-footprint[0] / 2, -footprint[1] / 2, 1],
        [-footprint[0] / 2, footprint[1] / 2, 1],
        [footprint[0] / 2, footprint[1] / 2, 1],
        [footprint[0] / 2, -footprint[1] / 2, 1],
    ]

    rotation = np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

    return np.dot(points, rotation.T)[:, :2]


def get_random_reference_points(center_line, count, start_idx=100, back_idx=500):
    random_stations = np.random.randint(start_idx, len(center_line.points) - back_idx, count)

    ref_s = []
    ref_x = []
    ref_y = []
    ref_theta = []
    for i in random_stations:
        ref_s.append(center_line.points[i].s)
        ref_x.append(center_line.points[i].x)
        ref_y.append(center_line.points[i].y)
        ref_theta.append(center_line.points[i].theta)
    return np.vstack([ref_s, ref_x, ref_y, ref_theta]).transpose()


def generate_random_vehicles(center_line, count, length=4.0, width=2.0):
    lateral_samples = np.array([1.0, 0.0, -4.0])
    random_laterals = lateral_samples[np.random.randint(0, len(lateral_samples), count)]

    ref = get_random_reference_points(center_line, count)
    ref_theta = ref[:, 3]
    ob_x, ob_y = convert_frenet_to_cartesian(ref, random_laterals)

    msg = Obstacles()
    for i in range(count):
        footprint = transform_footprint(ob_x[i], ob_y[i], ref_theta[i], [length, width])
        polygon = Polygon(points=[Point32(x=row[0], y=row[1]) for row in footprint])
        msg.obstacles.append(polygon)

    return msg


def generate_random_dynamic_vehicles(center_line, count, horizon=16.0, dt=0.1):
    cls = [pt.s for pt in center_line.points]
    max_s = cls[-1]
    ref = get_random_reference_points(center_line, count, back_idx=1000)
    ref_center_line = np.array([[pt.s, pt.x, pt.y, pt.theta] for pt in center_line.points])

    velocities = 4 + 2 * np.random.rand(count)
    footprint = transform_footprint(0, 0, 0, [4.0, 2.0])
    vehicle = Polygon(points=[Point32(x=row[0], y=row[1]) for row in footprint])

    msg = DynamicObstacles()
    for i in range(count):
        start_s = ref[i, 0]
        start_s_ind = bisect.bisect_left(cls, start_s)

        traj_len = int(horizon / dt)
        s_ind = np.linspace(start_s_ind, bisect.bisect_left(cls, min(max_s, start_s + velocities[i] * horizon)),
                            traj_len, dtype=np.int)

        rand_lateral = 0.0 if np.random.rand() > 0.5 else -4.0
        traj_x, traj_y = convert_frenet_to_cartesian(ref_center_line[s_ind, :], np.repeat(rand_lateral, traj_len))
        traj_theta = ref_center_line[s_ind, 3]

        tps = [DynamicTrajectoryPoint(time=j * dt, x=traj_x[j], y=traj_y[j], theta=traj_theta[j]) for j in
               range(traj_len)]
        msg.obstacles.append(DynamicObstacle(polygon=vehicle, trajectory=tps))

    return msg


def generate_random_pedestrian(center_line, count, dt=0.1, ego_velocity=20.0):
    ref = get_random_reference_points(center_line, count)

    velocities = 0.4 + np.random.rand(count)
    road_lb = -center_line.points[0].right_bound - 1.0
    road_ub = center_line.points[0].left_bound + 1.0
    distance = road_ub - road_lb

    pedestrian = Polygon(points=[
        Point32(x=-0.5, y=-0.5, z=0.0),
        Point32(x=-0.5, y=0.5, z=0.0),
        Point32(x=0.5, y=0.5, z=0.0),
        Point32(x=0.5, y=-0.5, z=0.0),
    ])

    msg = DynamicObstacles()
    for i in range(count):
        s = ref[i, 0]

        traj_len = int(distance / velocities[i] / dt)
        laterals = np.linspace(road_ub, road_lb, traj_len) if np.random.rand(1) > 0.5 else np.linspace(road_lb, road_ub,
                                                                                                       traj_len)

        traj_x, traj_y = convert_frenet_to_cartesian(ref[np.repeat(i, traj_len), :], laterals)

        time_offset = s / ego_velocity

        tps = [DynamicTrajectoryPoint(time=time_offset + j * dt, x=traj_x[j], y=traj_y[j], theta=0) for j in
               range(traj_len)]
        msg.obstacles.append(DynamicObstacle(polygon=pedestrian, trajectory=tps))

    return msg


def main():
    rospy.init_node('reference_publisher')

    config = [
        30,
        # degree, radius
        [-90, 10],
        10,
        [180, 5],
        36,
        [-180, 12],
        50,
    ]
    center_line = generate_center_line(config)
    center_pub = rospy.Publisher('/center_line', CenterLine, queue_size=1, latch=True)
    center_pub.publish(center_line)

    static_obstacles = None
    if 'static' in sys.argv:
        static_obstacles = generate_random_vehicles(center_line, 2)
        obstacles_pub = rospy.Publisher('/obstacles', Obstacles, queue_size=1, latch=True)
        obstacles_pub.publish(static_obstacles)

    dynamic_obstacles = None
    if 'dynamic' in sys.argv or 'pedestrian' in sys.argv:
        dynamic_obstacles = DynamicObstacles()
        if 'pedestrian' in sys.argv:
            dynamic_obstacles.obstacles.extend(generate_random_pedestrian(center_line, 6).obstacles)

        if 'dynamic' in sys.argv:
            dynamic_obstacles.obstacles.extend(generate_random_dynamic_vehicles(center_line, 3).obstacles)

        dynamic_obstacles_pub = rospy.Publisher('/dynamic_obstacles', DynamicObstacles, queue_size=1, latch=True)
        dynamic_obstacles_pub.publish(dynamic_obstacles)

    if 'serialize' in sys.argv:
        pickle_path = os.path.join(os.path.dirname(__file__), 'reference.pickle')
        with open(pickle_path, 'w') as f:
            pickle.dump({"center": center_line, "static": static_obstacles, "dynamic": dynamic_obstacles}, f)
            print('pickle saved to %s' % pickle_path)

    if 'path' in sys.argv:
        ref_pub = rospy.Publisher('/center_line_path', Path, queue_size=1, latch=True)
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        for pt in center_line.points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = pt.x
            pose.pose.position.y = pt.y
            msg.poses.append(pose)

        ref_pub.publish(msg)

    rospy.spin()


if __name__ == '__main__':
    main()
