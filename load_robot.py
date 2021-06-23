import pybullet as p
import numpy as np
import time
import pybullet_data
from robot import InvertedPendulum
from robot import Bipedal
import tform as tf


robot = InvertedPendulum();
robot.torqueControllModeEnableForAll()
while(1):

	q=robot.getJointPositions();
	dq=robot.getJointVelocities();
	
	u = [0,0,0]
	robot.setBaseVelocities([u[0]])		
	robot.setArmJointTorques([u[1],u[2]])
	T=robot.forwardKinematics(q)
	
	robot.oneStep();
