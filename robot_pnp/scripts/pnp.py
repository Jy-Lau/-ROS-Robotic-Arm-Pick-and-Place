#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospkg
import yaml
from actionlib_msgs.msg import GoalStatusArray
import moveit_commander
from moveit_commander import MoveGroupCommander
from control_msgs.msg import JointTrajectoryControllerState

class PnP():

    def __init__(self):
        rospy.init_node('pnp_node', anonymous=True)  
        rospy.loginfo("Pick and Place...")
        # self._check_moveit_ready()
        self._check_controller_ready()
        self.pub_arm = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.pub_gripper = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
        # self.camera_subscriber = rospy.Subscriber('/plan', String, self.camera_callback)
        self.rate = rospy.Rate(10)
        self._setup_arm_gripper()
        # move_group = MoveGroupCommander('arm')
        # # Set the planning time for the arm
        # move_group.set_planning_time(10)

        # # Set the target pose for the end-effector
        # target_pose = move_group.get_current_pose().pose
        # target_pose.position.x += 0.1

        # # Plan and execute the trajectory to the target pose
        # move_group.set_pose_target(target_pose)
        # plan = move_group.go(wait=True)
        # move_group.stop()
        # move_group.clear_pose_targets()
        
        # self.hand_subscriber = rospy.Subscriber('/plan', Int32MultiArray, self.hand_callback)

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
        # elif msg.data == 'Up':
        #     self.arm_point.positions[1] = max(self.arm_point.positions[1]-0.01,-1.13) # shoulder_lift_joint
        # elif msg.data == 'Down':
        #     self.arm_point.positions[1] = min(self.arm_point.positions[1]+0.01,-0.75) # shoulder_lift_joint
        # elif msg.data == 'Left':
        #     self.arm_point.positions[2] = min(self.arm_point.positions[2]+0.01, 0.25) # shoulder_pan_joint
        # elif msg.data == 'Right':
        #     self.arm_point.positions[2] = max(self.arm_point.positions[2]-0.01,-0.88) # shoulder_pan_joint

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
    while not rospy.is_shutdown():
        try:
            pnp.publish_joint_traj()
            pnp.rate.sleep()
        except rospy.ROSInterruptException:
            pass



