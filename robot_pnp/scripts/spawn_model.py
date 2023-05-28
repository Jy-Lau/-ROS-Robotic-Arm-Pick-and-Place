#!/usr/bin/env python3

import rospy, tf, rospkg, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point
import random
import threading
from robot_pnp.srv import NumModel
from std_msgs.msg import Int32

class CubeSpawner():

	def __init__(self):
		self.rospack = rospkg.RosPack()
		self.path = self.rospack.get_path('robot_pnp')+"/urdf/"
		self.cubes = []
		# self.cubes.append(self.path+"red_cube.urdf")
		# self.cubes.append(self.path+"green_cube.urdf")
		self.cubes.append(self.path+"blue_cube.urdf")
		self.counter = 0
		self.live_cubes =[]

		self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
		self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
		self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
		self.model_pub = rospy.Publisher('/model/count', Int32, queue_size=10)
		rospy.Service('num_model', NumModel, self.num_model)

	def num_model(self,msg):
		self._spawn_model(msg.count)
		return True

	def checkModel(self,cube):
		res = self.ms(cube, "world")
		return res.success

	def getPosition(self,cube):
		res = self.ms(cube, "world")
		return res.pose.position.y

	def _spawn_model(self,count):
		cube = random.choice(self.cubes)
		with open(cube,"r") as f:
			cube_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(0,0,0)
		orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
		pose = Pose(Point(x=1.7,y=5.2,z=1), orient)
		self.sm(f"cube{self.counter}", cube_urdf, '', pose, 'world')
		self.live_cubes.append(f"cube{self.counter}")
		self.counter +=1
		msg = Int32()
		msg.data = 1
		self.model_pub.publish(msg)

	def deleteModel(self,cube):
		self.dm(cube)
	
	def deleteAllModel(self, cubes):
		if cubes != None and len(cubes) !=0:
			for cube in cubes:
				self.dm(cube)

	def shutdown_hook(self):
		self.deleteAllModel(self.live_cubes)
		rospy.loginfo("Shutting down")


if __name__ == "__main__":
	rospy.loginfo("Waiting for gazebo services...")
	rospy.init_node("spawn_cubes")
	rospy.wait_for_service("/gazebo/delete_model")
	rospy.wait_for_service("/gazebo/spawn_urdf_model")
	rospy.wait_for_service("/gazebo/get_model_state")
	rospy.loginfo("Waiting for gazebo services...Done")
	cs = CubeSpawner()
	rospy.on_shutdown(cs.shutdown_hook)
	rospy.spin()