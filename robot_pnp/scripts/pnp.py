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
        self.skip=True
        self.rate = rospy.Rate(10)
        self._check_controller_ready()
        self.pub_arm = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.pub_gripper = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
        self._setup_arm_gripper()
        self._check_moveit_ready()
        self._setup_move_group()
        rospy.Service('robot_pnp', PointArray, self.handle_item)
    
    def _setup_move_group(self):
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.move_group = MoveGroupCommander(self.robot.get_group_names()[0])
        self.move_group.set_goal_tolerance(0.01)
        self.move_group.set_planning_time(1.5) #Set the planning time for the arm
        # self.move_group.set_num_planning_attempts(5)
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.set_start_state_to_current_state()
        joint_goal = dict(zip(self.arm_traj.joint_names, self.arm_point.positions))
        rospy.sleep(2)
        self.move_group.set_joint_value_target(joint_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def handle_item(self,req):
        self.skip=False
        rospy.loginfo(f'From PointCloud[0]: {req.points[0]}')
        # Set the target pose for the end effector
        pose = Pose()
        pose.position.x = req.points[0].point.x
        pose.position.y = req.points[0].point.y
        pose.position.z = req.points[0].point.z + 0.19 #0.19 is size of wooden block + part of gripper
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

        #Open Gripper
        rospy.loginfo(f'Opening Gripper')
        start_time = rospy.Time.now()
        duration = rospy.Duration.from_sec(0.5)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = current_time - start_time
            self.gripper_traj.header.stamp = current_time
            self.gripper_traj.points = [self.gripper_point]
            self.pub_gripper.publish(self.gripper_traj)
            if elapsed_time >= duration:
                break
        rospy.loginfo(f'Closing Gripper')
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
            if elapsed_time >= duration:
                break

        return success

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
        

    def camera_callback(self, msg):
        if msg.data == 'Open':
            self.gripper_point.positions = self.config['gripper_open_positions']
        elif msg.data == 'Close':
            self.gripper_point.positions = self.config['gripper_close_positions']

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

    def publish_joint_traj(self):
        if(self.skip):
            self.arm_traj.header.stamp = rospy.Time.now()
            self.arm_traj.points = [self.arm_point]
            self.pub_arm.publish(self.arm_traj)

            self.gripper_traj.header.stamp = rospy.Time.now()
            self.gripper_traj.points = [self.gripper_point]
            self.pub_gripper.publish(self.gripper_traj)
    
    def _construct_trajectory_point(self, joint_traj, posture, duration):
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = rospy.Duration.from_sec(float(duration))
        for key in joint_traj.joint_names:
            trajectory_point.positions.append(posture[key])
        return trajectory_point

    def _run(self, traj):
        #trajectory_start_time = 1.0
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration.from_sec(float(trajectory_start_time))
        joint_trajectory.joint_names = list(traj.keys())
        joint_trajectory.points = []
        this_trajectory_point = self._construct_trajectory_point(joint_trajectory, traj, 0.05)
        joint_trajectory.points.append(this_trajectory_point)
        self.hand_commander.run_joint_trajectory_unsafe(joint_trajectory, False)

if __name__ == '__main__':
    pnp = PnP()
    rospy.spin()
    # while not rospy.is_shutdown():
    #     try:
    #         pnp.publish_joint_traj()
    #         pnp.rate.sleep()
    #     except rospy.ROSInterruptException:
    #         pass



