#!/usr/bin/env python3
"""
Shows how to use a planning scene in MoveItPy to add collision objects and perform collision checking.
"""
import pandas as pd
import copy
import time
import rclpy
from rclpy.logging import get_logger
import spatialmath.base as smb
import time

from moveit.planning import MoveItPy, PlanRequestParameters

from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg

from geometry_msgs.msg import PoseStamped
import moveit_msgs

import frankie_planner.moveit_utils as utils


def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py_planning_scene")

    planning_interface = utils.get_planning_interface()

    pose_node = rclpy.create_node("pose_publisher")
    pose_pub = pose_node.create_publisher(PoseStamped, "debug_pose", 10)
    robot_state_pub = pose_node.create_publisher(moveit_msgs.msg.DisplayRobotState, "ik_sol", 10)
    robot = planning_interface.get_planning_component("mobile_base_arm")
    robot.set_workspace(-0,-1.0,-1.,1.,2.5,3.)
    logger.info("MoveItPy instance created")

    planning_scene_monitor = planning_interface.get_planning_scene_monitor()

    package_path = __import__("frankie_planner").__path__[0] + "/../"

    working = 0 # To have a rolling avg window

    episode_template = {
        "name": "",
        "solved" : False,
        "IK_total": 0,
        "IK_avg": 0,
        "plan_tries" : 0,
        "wall_time" : 0,
    }
    exp_data = []


    for i in range(500):
        data = copy.deepcopy(episode_template)
        robot.set_start_state(configuration_name="home")
        exp_name = f"bookshelf_cage_{i:04d}"

        data["name"] = exp_name
        target_pose = utils.load_rmmi_scene(planning_scene_monitor, f"{package_path}/data/bookshelf_cage/{exp_name}.json", True)

        robot_state = RobotState(planning_interface.get_robot_model())
        # Set the pose goal
        pose_goal = utils.sm_to_ros(target_pose)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "odom"  # Ensure this matches your TF setup
        pose_msg.header.stamp = pose_node.get_clock().now().to_msg()
        pose_msg.pose = pose_goal

        pose_pub.publish(pose_msg)

        ti = time.perf_counter()
        solved = False
        for plan_tries in range(100):
            ik_solved = False
            total_ik_tries = 0
            for ik_tries in range(1000):
                total_ik_tries += 1
                with planning_scene_monitor.read_only() as scene:
                    ik_sol = robot_state.set_from_ik("mobile_base_arm", pose_goal, "frankie_hand", 0.5)
                    if not ik_sol:
                        continue
                    robot_state.update()  # required to update transforms

                    robot_collision_status = scene.is_state_colliding(
                        robot_state=robot_state, joint_model_group_name="mobile_base_arm", verbose=False
                    )
                    robot_msg = utils.convert_to_display_robot_state(robotStateToRobotStateMsg(robot_state))

                    robot_state_pub.publish(robot_msg)
                # print(f"IK {ik_tries} resulted on {robot_collision_status} collision")
                if not robot_collision_status:
                    ik_solved = True
                    print(f"Sampled a valid IK after {ik_tries + 1}")
                    break
            if ik_solved: # We try to plan
                robot.set_start_state(configuration_name="home")
                robot.set_goal_state(robot_state = robot_state)


                plan = utils.plan(robot, logger)
                if plan:
                    input("Press enter to continue")
                    working += 1
                    solved = True
                    print(f"Problem {exp_name} solved, after ik:{total_ik_tries} and plan tries {plan_tries}")
                    break

        data["solved"] = solved
        data["IK_total"] = total_ik_tries
        data["IK_avg"] = total_ik_tries/(plan_tries + 1)
        data["plan_tries"] = plan_tries + 1
        data["wall_time"] = time.perf_counter() - ti

        print(f"Problem {exp_name} solved {solved}, after ik:{total_ik_tries} and plan tries {plan_tries}")
        print(data)
        exp_data.append(data)

        print(f"In total we obtained {working} / {i+1}")
    df = pd.DataFrame(exp_data)
    print(df)
    df.to_csv("bookshelf_results.csv")


if __name__ == "__main__":
    main()
