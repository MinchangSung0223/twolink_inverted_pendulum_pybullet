import pybullet as p
import numpy as np
import time
import pybullet_data
from robot import InvertedPendulum
from robot import Bipedal
import tform as tf


robot = InvertedPendulum();
robot.torqueControllModeEnableForAll()

prev_dq =[0,0,0]
dt = 1/240.0
u = [0,0,0]
kpCartId = p.addUserDebugParameter("kpCart", 0, 1300, 1000)
kdCartId = p.addUserDebugParameter("kdCart", 0, 1000, 200)
kiCartId = p.addUserDebugParameter("kiCart", 0, 10000, 0)

kpPoleId = p.addUserDebugParameter("kpPole", 0, 1300, 1000)
kdPoleId = p.addUserDebugParameter("kdPole", 0, 1000, 200)
kiPoleId = p.addUserDebugParameter("kiPole", 0, 10000, 0)

#qd_=robot.IKinSpace(np.array([[0 ,0 ,1,0.7],[0 ,1,0,0],[-1,0 ,0,0.8],[0 ,0 ,0,1]]),np.array([0,0,0]))
#qd_=qd_[0]
#qd = np.array([qd_[0],qd_[1],qd_[2]])
qd = np.array([0,0,0])
dqd = np.array([0,0,0.0])
ddqd = np.array([0,0,0])
sum_e=0;
sum_u0=0;
while(1):
	kpCart = p.readUserDebugParameter(kpCartId)
	kdCart = p.readUserDebugParameter(kdCartId)
	kiCart = p.readUserDebugParameter(kiCartId)
	kpPole = p.readUserDebugParameter(kpPoleId)
	kdPole = p.readUserDebugParameter(kdPoleId)
	kiPole = p.readUserDebugParameter(kiPoleId)
	Kp = np.diag([kpCart,kpPole,kpPole])
	Ki = np.diag([kiCart,kiPole,kiPole])
	Kd = np.diag([kdCart,kdPole,kdPole])
	
	
	
	q=robot.getJointPositions();
	dq=robot.getJointVelocities();
	e=qd-q;
	edot = dqd-dq;
	sum_e = sum_e+e*dt;


	T=robot.FKinSpace(q)
	J=robot.JacobianSpace
	M=robot.getMassMatrix(q)
	G=robot.getGravityAndCoriolis(q,dq,[0,0,0])
	
	p_term = Kp.dot(e - dq*dt)
	d_term = Kd.dot(edot)
	qddot = np.linalg.solve(a=(M + Kd * dt),b=p_term + d_term - G)
	u = M.dot(ddqd+Kp.dot(e)+Ki.dot(sum_e)+Kd.dot(edot))+G
	robot.setArmJointTorques([u[1],u[2]])
	sum_u0=0
	
	#robot.setBaseVelocities([kpCart*e[0]+kdCart*edot[0]+G[0]])			
	robot.setBaseJointTorques([u[0]])
	print("error",e)
	print("u",u)
	
	robot.oneStep();

