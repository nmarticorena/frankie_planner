import json

import spatialmath as sm
import spatialmath.base as smb

# Ros imports
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, DisplayRobotState
from shape_msgs.msg import SolidPrimitive
from moveit_configs_utils import MoveItConfigsBuilder

from moveit.planning import MoveItPy


def plan(
    planning_component,
    logger,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    plan_result = planning_component.plan()

    if plan_result:
        logger.info("Plan success")
        return True
    else:
        logger.error("Planning failed")
        return False


def load_rmmi_scene(planning_scene_monitor, file_path:str, collisions = True)-> sm.SE3:
    """Helper function that adds collision objects to the planning scene."""
    with open(file_path, 'r') as file:
        data = json.load(file)

    boxes = data["cubes"]
    cylinders = data["cylinders"]


    target_pose = sm.SE3(data["target"], check = False) * sm.SE3.Tz(-0.1034)

    if not collisions:
        return target_pose

    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "odom"
        collision_object.id = "boxes"

        for position, dimensions in zip(
                (b["position"] for b in boxes), (b["scale"] for b in boxes)
            ):
            box_pose = Pose()
            box_pose.position.x = position[0]
            box_pose.position.y = position[1]
            box_pose.position.z = position[2]
            dimensions = [d * 2 for d in dimensions]


            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD
        for position, r , h in zip(
                (c["position"] for c in cylinders), (c["radius"] for c in cylinders), (c["heigth"] for c in cylinders)
            ):
            box_pose = Pose()
            box_pose.position.x = position[0]
            box_pose.position.y = position[1]
            box_pose.position.z = position[2]


            box = SolidPrimitive()
            box.type = SolidPrimitive.CYLINDER
            box.dimensions = [h * 2, r]

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()  # Important to ensure the scene is updated

    return target_pose

def convert_to_display_robot_state(robot_state_msg):
    """
    Converts a moveit::core::RobotState to moveit_msgs::DisplayRobotState.

    :param robot_state: The MoveIt RobotState object
    :param robot_model: The MoveIt RobotModel object (used for visualizing)
    :return: Corresponding moveit_msgs.msg.DisplayRobotState message
    """
    # Convert to moveit_msgs::RobotState
    # Create DisplayRobotState
    display_robot_state_msg = DisplayRobotState()
    display_robot_state_msg.state = robot_state_msg

    return display_robot_state_msg

def sm_to_ros(pose: sm.SE3):
    pose_msg = Pose()
    x,y,z = pose.t
    pose_msg.position.x = x
    pose_msg.position.y = y
    pose_msg.position.z = z
    q = smb.r2q(pose.R, order = "xyzs")
    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]

    return pose_msg

def get_planning_interface() -> MoveItPy:
    moveit_config = (
        MoveItConfigsBuilder(robot_name="frankie", package_name="frankie_moveit_config")
        .robot_description_semantic(file_path="config/frankie.srdf")
        .robot_description(file_path="config/frankie.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/copy_controllers.yaml")
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
    ).to_moveit_configs().to_dict()
    import json
    with open("moveit_config.json", "w") as f:
        json.dump(moveit_config, f, indent=4)
    return MoveItPy(node_name="moveit_py", config_dict = moveit_config)

