#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospkg
import yaml
from actionlib_msgs.msg import GoalStatusArray
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from control_msgs.msg import JointTrajectoryControllerState
from robot_pnp.srv import PointArray
from geometry_msgs.msg import Pose
import copy
from moveit_msgs.msg import RobotState
from tf.transformations import quaternion_from_euler
import math
class PnP():

    def __init__(self):
        rospy.init_node('pnp_node', anonymous=True)  
        rospy.loginfo("Pick and Place...")
        self.rate = rospy.Rate(10)
        self._check_controller_ready()
        self.pub_arm = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.pub_gripper = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
        self._setup_arm_gripper()
        self._check_moveit_ready()
        self._setup_move_group()
        self._move_to_home()
        rospy.Service('robot_pnp', PointArray, self.handle_item)
    
    def _setup_move_group(self):
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.move_group = MoveGroupCommander(self.robot.get_group_names()[0])
        self.move_group.set_goal_tolerance(0.01)
        self.move_group.set_planning_time(5) #Set the planning time for the arm
        self.move_group.set_num_planning_attempts(5)
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.set_start_state_to_current_state()
        rospy.sleep(2)
    

    def _setup_arm_gripper(self):
        self.config = {}
        with open(rospkg.RosPack().get_path('robot_pnp') + f"/config/robot_control.yaml", 'r') as f:
            self.config = yaml.safe_load(f)
        self.arm_traj = JointTrajectory()
        self.arm_traj.joint_names = self.config['arm_joint_names']
        self.arm_point = JointTrajectoryPoint()
        self.arm_point.positions = self.config['arm_scanning_positions']
        self.arm_point.time_from_start = rospy.Duration.from_sec(0.25)
        
        self.gripper_traj = JointTrajectory()
        self.gripper_traj.joint_names = self.config['gripper_joint_names']
        self.gripper_point = JointTrajectoryPoint()
        self.gripper_point.positions = self.config['gripper_open_positions']
        self.gripper_point.time_from_start = rospy.Duration.from_sec(0.25)

        self.gripper_duration = rospy.Duration.from_sec(1.5)

    def _check_moveit_ready(self):
        moveit_msg = None
        rospy.loginfo("Checking Moveit...")
        while moveit_msg is None and not rospy.is_shutdown():
            try:
                moveit_msg = rospy.wait_for_message("/move_group/status", GoalStatusArray, timeout=1.0)
                rospy.logdebug("Current /move_group/status READY=>" + str(moveit_msg))

            except:
                rospy.logerr("Current /move_group/status not ready yet, retrying for getting moveit")
        rospy.loginfo("Checking Moveit...DONE")

    def _check_controller_ready(self):
        controller_state = None
        rospy.loginfo("Checking Controller...")
        while controller_state is None and not rospy.is_shutdown():
            try:
                controller_state = rospy.wait_for_message("/eff_joint_traj_controller/state", JointTrajectoryControllerState, timeout=1.0)
                rospy.logdebug("Current /eff_joint_traj_controller/state READY=>" + str(controller_state))

            except:
                rospy.logerr("Current /eff_joint_traj_controller/state not ready yet, retrying for getting controller")
        
        controller_state = None
        while controller_state is None and not rospy.is_shutdown():
            try:
                controller_state = rospy.wait_for_message("/gripper_controller/state", JointTrajectoryControllerState, timeout=1.0)
                rospy.logdebug("Current /gripper_controller/state READY=>" + str(controller_state))

            except:
                rospy.logerr("Current /gripper_controller/state not ready yet, retrying for getting controller")

        rospy.loginfo("Checking Controller...DONE")

    def _move_to_home(self):
        rospy.loginfo(f'Moving to Home')
        joint_goal = dict(zip(self.arm_traj.joint_names, self.arm_point.positions))
        self.move_group.set_joint_value_target(joint_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo(f'Done moving to Home')

    def _move_to_target(self):
        pose = Pose()
        pose.position.x = 0
        pose.position.y = -0.6
        pose.position.z = 0.6 #0.19 is size of wooden block + part of gripper
        # current_pose = self.move_group.get_current_pose().pose
        quat = quaternion_from_euler(math.pi, 0, 0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.move_group.set_pose_target(pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        pose = Pose()
        pose.position.x = 0
        pose.position.y = -0.6
        pose.position.z = 0.2 #0.19 is size of wooden block + part of gripper
        # current_pose = self.move_group.get_current_pose().pose
        quat = quaternion_from_euler(math.pi, 0, 0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.move_group.set_pose_target(pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def _open_gripper(self):
        rospy.loginfo(f'Opening gripper')
        start_time = rospy.Time.now()
        self.gripper_point = JointTrajectoryPoint()
        self.gripper_point.positions = self.config['gripper_open_positions']
        self.gripper_point.time_from_start = rospy.Duration.from_sec(0.25)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = current_time - start_time
            self.gripper_traj.header.stamp = current_time
            self.gripper_traj.points = [self.gripper_point]
            self.pub_gripper.publish(self.gripper_traj)
            if elapsed_time >= self.gripper_duration:
                break
        rospy.loginfo(f'Done opening gripper')

    def _close_gripper(self):
        rospy.loginfo(f'Closing gripper')
        start_time = rospy.Time.now()
        self.gripper_point = JointTrajectoryPoint()
        self.gripper_point.positions = self.config['gripper_close_positions']
        self.gripper_point.time_from_start = rospy.Duration.from_sec(0.25)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = current_time - start_time
            self.gripper_traj.header.stamp = current_time
            self.gripper_traj.points = [self.gripper_point]
            self.pub_gripper.publish(self.gripper_traj)
            if elapsed_time >= self.gripper_duration:
                break
        rospy.loginfo(f'Done closing gripper')

    def handle_item(self,req):
        rospy.loginfo(f'From PointCloud[0]: {req.points[0]}')
        # Set the target pose for the end effector
        pose = Pose()
        pose.position.x = req.points[0].point.x
        pose.position.y = req.points[0].point.y
        pose.position.z = req.points[0].point.z + 0.2 #0.2 is size of wooden block + part of gripper
        quat = quaternion_from_euler(math.pi, 0, 0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.move_group.set_pose_target(pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        self._close_gripper()
        self._move_to_target()
        self._open_gripper()
        self._move_to_home()

        return True

if __name__ == '__main__':
    pnp = PnP()
    rospy.spin()



