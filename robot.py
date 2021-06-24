import pybullet as pb
import pybullet_data
import numpy as np
import time
import tform as tf
import scipy.linalg as la
import modern_robotics as mr
class Robot:
    def __init__(self, robotPATH, startPosition, startOrientation, maxForce, controlMode=pb.POSITION_CONTROL, planePATH="plane.urdf",timeStep=1/240.0):
        try:
            physicsClient = pb.connect(pb.GUI)
        except:
            pass
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0,0,-9.806)
        #self._planeId = pb.loadURDF(planePATH)
        self._robotId = pb.loadURDF(robotPATH,startPosition, pb.getQuaternionFromEuler(startOrientation))
        self._controlMode = controlMode
        self.numJoint = pb.getNumJoints(self._robotId)
        self._jointIdList = list(range(self.numJoint))

        self.maxForce = maxForce
        self._maxForceList = [maxForce]*12

        self._timeStep = timeStep


    def getEuler(self):
        _, qua = pb.getBasePositionAndOrientation(self._robotId)
        return pb.getEulerFromQuaternion(qua)

    def getQuaternion(self):
        _, orientation = pb.getBasePositionAndOrientation(self._robotId)
        return orientation

    def getRobotPosition(self):
        position, _ = pb.getBasePositionAndOrientation(self._robotId)
        return position

    def resetRobotPositionAndOrientation(self, position, orientation):
        pb.resetBasePositionAndOrientation(self._robotId, position, orientation)

    def setMotorTorqueByArray(self, targetJointTorqueList):
        if self._controlMode is pb.TORQUE_CONTROL:
            pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdList, controlMode=pb.TORQUE_CONTROL, forces=targetJointTorqueList)
        else:
            print("Error: Mode must be set to TORQUE MODE")
        
    def enableJointForceTorque(self,joint_number):
        print(self._jointIdListR)
        pb.enableJointForceTorqueSensor(self._robotId, jointIndex=joint_number)
    def getJointForceTorque(self,joint_number):
        val=pb.getJointState(self._robotId, jointIndex=joint_number)
        return val[2]

     
    def setMotorPositionByArray(self, targetJointPositionList):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdList, controlMode=self._controlMode, forces=self._maxForceList, targetPositions=targetJointPositionList)

    def oneStep(self):
        
        robotPosition, _ = pb.getBasePositionAndOrientation(self._robotId)
        #pb.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=135, cameraPitch=-10, cameraTargetPosition=robotPosition)
        pb.stepSimulation()
        time.sleep(self._timeStep)


class Bipedal(Robot):

    def __init__(self, startPosition=[0,0,0.55], startOrientation=[0,0,0], CoMposition_b=np.array([0.,0.,-0.02]),maxForce=9.0, controlMode=pb.POSITION_CONTROL, robotPATH="urdf/bipedal.urdf", planePATH="plane.urdf",timeStep=1/240.0):
        super().__init__(robotPATH, startPosition, startOrientation, maxForce, controlMode=controlMode, planePATH=planePATH,timeStep=timeStep)

        self._lambda = 1.0
        self._L1 = 0.18
        self._L2 = 0.18
        self.R = np.array([0,-0.065,-0.175]) - CoMposition_b #from CoM to hipyaw joint
        self.L = np.array([0,0.065,-0.175]) - CoMposition_b
        self.LEG_DOF = 6

        self._jointIdListR = [0,1,2,3,4,5]
        self._jointIdListL = [6,7,8,9,10,11]
        self._maxForceListForLeg = [maxForce]*self.LEG_DOF

        #joint axis matrix
        self._a = np.array([[0,0,1], 
                                [1,0,0],
                                [0,1,0],
                                [0,1,0],
                                [0,1,0],
                                [1,0,0]])

        self._E = np.eye(3)



    def setRightLegJointPositions(self, targetJointPositions):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdListR, controlMode=self._controlMode, forces=self._maxForceListForLeg, targetPositions=targetJointPositions)

    def setLeftLegJointPositions(self, targetJointPositions):
        pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdListL, controlMode=self._controlMode, forces=self._maxForceListForLeg, targetPositions=targetJointPositions)

    

    def torqueControllModeEnableForAll(self):

        pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointIdList, controlMode=pb.VELOCITY_CONTROL, forces=[0]*12)
        self._controlMode = pb.TORQUE_CONTROL
        
        
    def getLegTrans(self, jointPositions, leg):
        hipyaw = jointPositions[0]
        hiproll = jointPositions[1]
        hippitch = jointPositions[2]
        knee = jointPositions[3]
        anklepitch = jointPositions[4]
        ankleroll = jointPositions[5]
        zero_v = np.zeros(3)

        T_0_1 = tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[0], hipyaw),leg)
        T_0_2 = T_0_1.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[1], hiproll), zero_v))
        T_0_3 = T_0_2.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[2], hippitch), zero_v))
        T_0_4 = T_0_3.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[3], knee), [0,0,-self._L1]))
        T_0_5 = T_0_4.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[4], anklepitch), [0,0,-self._L2]))
        T_0_6 = T_0_5.dot( tf.getTransFromRp(tf.roadriguesEquation(self._E, self._a[5], ankleroll), zero_v))

        return T_0_1, T_0_2, T_0_3, T_0_4, T_0_5, T_0_6
        
        
        
    def forwardKinematics(self, jointPositions, leg):

        T_0_6 = self.getLegTrans(jointPositions, leg)[5]

        return tf.getRotationAndPositionFromT(T_0_6)

    def inverseKinematics(self, p_ref, omega_ref, leg):
        q = self.getJointPositions(leg)
        R, p = self.forwardKinematics(q, leg)
        omega = np.array(tf.getRollPitchYawFromR(R))

        dp = p_ref - p
        domega = omega_ref - omega
        dp_domega = np.append(dp,domega)

        dq = self._lambda * la.inv(self.jacobian(q, leg)).dot(dp_domega)


        return q+dq

    def jacobian(self, q, leg):
        T0 = self.getLegTrans(q, leg)
        zero_v = np.zeros(3)

        R = [tf.getRotationFromT(T0[i]) for i in range(len(T0))]
        p = [tf.getPositionFromT(T0[i]) for i in range(len(T0))]

        wa = [R[i].dot(self._a[i]) for i in range(len(R))]

        Jp = np.vstack((  np.hstack((np.cross( wa[i],(p[5]-p[i])), wa[i])) for i in range(len(wa)-1)))
        J = np.vstack(( Jp, np.hstack((zero_v, wa[5])) )).T

        return J

    def getJointPositions(self, leg):
        if np.sum(leg == self.R) == len(leg):
            jointStates = pb.getJointStates(self._robotId, jointIndices=self._jointIdListR)
            jointPositions = [jointStates[i][0] for i in range(len(jointStates))]

        elif np.sum(leg == self.L) == len(leg):
            jointStates = pb.getJointStates(self._robotId, jointIndices=self._jointIdListL)
            jointPositions = [jointStates[i][0] for i in range(len(jointStates))]
        
        else:
            raise ValueError("invalid parameter")

        return jointPositions
        
class InvertedPendulum(Robot):
	def __init__(self, startPosition=[0,0,-0.1], startOrientation=[0,0,0], CoMposition_b=np.array([0.,0.,-0.02]),maxForce=9.0, controlMode=pb.POSITION_CONTROL, robotPATH="urdf/mobile_manipulator.urdf", planePATH="plane.urdf",timeStep=1/240.0):
		super().__init__(robotPATH, startPosition, startOrientation, maxForce, controlMode=controlMode, planePATH=planePATH,timeStep = timeStep)        
		self._jointList = [0,3,4];
		self._armjointList = [3,4];
		self._basejointList=[0];
		self._FTsensorJoint = [1];
		self._maxForce = maxForce;
		self._Kp = [500,500,500];
		self._Kd = [150,150,150];
		self._timeStep=timeStep
		W1 = 0.2
		H1 = 0.05
		H2 = 0.25
		L1 = 0.5;
		L2 = 0.5;
		
		self.M =   np.array([[1,0,0,W1],
				[0,1,0,0],
				[0,0,1,L1+L2+H1+H2],
				[0,0,0,1]])
		M01=  np.array([[1, 0, 0,        0],
				[0, 1, 0,        0],
				[0, 0, 1,        0],
				[0, 0, 0,        1]])
		M12=  np.array([[1, 0, 0,        W1],
				[0, 1, 0,        0],
				[0, 0, 1, H1+H2+L1/2],
				[0, 0, 0,        1]])
		M23=  np.array([[1, 0, 0,        0],
				[0, 1, 0,        0],
				[0, 0, 1, L1/2+L2/2],
				[0, 0, 0,        1]])
		M34=  np.array([[1, 0, 0,        0],
				[0, 1, 0,        0],
				[0, 0, 1, L2/2],
				[0, 0, 0,        1]])
		G1 = np.diag([1,1,1,10*9.8,10*9.8,10*9.8])
		G2 = np.diag([1,1,1,2*9.8,2*9.8,2*9.8])
		G3 = np.diag([1,1,1,2*9.8,2*9.8,2*9.8])	
		self.Glist=np.array([G1,G2,G3])
		self.Mlist=np.array([M01,M12,M23,M34])
		self.g = np.array([0,0,-9.8])
		print(np.matmul(np.matmul(np.matmul(M01,M12),M23),M34))
		self.Slist = np.array([[0,0,0,1,0,0],[0,1,0,-(H1+H2),0,W1],[0,1,0,-(H1+H2+L1),0,W1]]).T
	def torqueControllModeEnableForAll(self):
		pb.setJointMotorControlArray(self._robotId, jointIndices=self._jointList, controlMode=pb.VELOCITY_CONTROL, forces=[0]*len(self._jointList))
		self._controlMode = pb.TORQUE_CONTROL
	def IKinSpace(self, T,jointPositions0,eomg=0.01,ev=0.001):               
		jointPositions=mr.IKinSpace(self.Slist,self.M,T,jointPositions0,eomg,ev)
		return jointPositions
	def FKinSpace(self, jointPositions):
		thetalist = np.array([jointPositions[0],jointPositions[1],jointPositions[2]])		
		T_eef = mr.FKinSpace(self.M,self.Slist,thetalist)
		return T_eef
	def JacobianSpace(self, jointPositions):
		thetalist = np.array([jointPositions[0],jointPositions[1],jointPositions[2]])		
		J = mr.JacobianSpace(self.Slist,thetalist)
		return J
	def inverseDynamics(self, jointPositions,jointVelocities,targetAccelerations):
		torque = [0,0,0]
		return torque 
	def ForwardDynamics(self,jointPositions,jointVelocities,torques,Ftip):
		thetalist = np.array([jointPositions[0],jointPositions[1],jointPositions[2]])	
		dthetalist = np.array([jointVelocities[0],jointVelocities[1],jointVelocities[2]])
		taulist = np.array([torques[0],torques[1],torques[2]])
		Ftip_ = np.array([Ftip[0],Ftip[1],Ftip[2],Ftip[3],Ftip[4],Ftip[5]])
		jointAccelrations = mr.ForwardDynamics(thetalist,dthetalist,taulist,self.g,Ftip.T,self.Mlist,self.Glist,self.Slist)
		return jointAccelrations
	def EulerStep(self,thetalist,dthetalist,ddthetalist,dt,intRes):
		return mr.EulerStep(thetalist, dthetalist,ddthetalist, 1.0 * dt / intRes)		
	def getJointPositions(self):
		jointStates = pb.getJointStates(self._robotId, jointIndices=self._jointList)
		jointPositions = [jointStates[0][0],jointStates[1][0],jointStates[2][0]]
		return np.array(jointPositions)
	def getJointVelocities(self):
		jointStates = pb.getJointStates(self._robotId, jointIndices=self._jointList)
		jointVelocities = [jointStates[0][1],jointStates[1][1],jointStates[2][1]]	
		return np.array(jointVelocities)
	def setEnableFTsensor(self):
		pb.enableJointForceTorqueSensor(self._robotId, jointIndex=self._FTsensorJoint)
	def setJointState(self,jointPositions):
		pb.resetJointState(self._robotId,0,jointPositions[0])		
		pb.resetJointState(self._robotId,3,jointPositions[1])		
		pb.resetJointState(self._robotId,4,jointPositions[2])		
		
	def getFTsensorValues(self):
		val=pb.getJointState(self._robotId, jointIndex=self._FTsensorJoint)
		FTsensorValues =val[2];
		return FTsensorValues
	def getMassMatrix(self,jointPositions):
		q = [jointPositions[0],jointPositions[1],jointPositions[2]]
		M=pb.calculateMassMatrix(self._robotId,q)
		return np.array(M)
	def getGravityAndCoriolis(self,jointPositions,jointVelocities,targetAccelerations):
		q = [jointPositions[0],jointPositions[1],jointPositions[2]]
		dq = [jointVelocities[0],jointVelocities[1],jointVelocities[2]]
		target_ddq = [targetAccelerations[0],targetAccelerations[1],targetAccelerations[2]]
		G = pb.calculateInverseDynamics(self._robotId, q, dq, target_ddq)
		return np.array(G)
				
	def getLinkState(self,linkIndex):
		val=pb.getLinkState(self._robotId, linkIndex=linkIndex)
		return val[0]
	def setArmJointTorques(self,taus):
		n = 0;
		for j  in self._armjointList:
			pb.setJointMotorControlMultiDof(self._robotId,j, controlMode=pb.TORQUE_CONTROL,force=[taus[n]])
			n = n+1;
	def setBaseVelocities(self,targetVels):
		pb.setJointMotorControlArray(self._robotId,self._basejointList, controlMode=pb.VELOCITY_CONTROL,targetVelocities = targetVels,forces=[self._maxForce]*len(self._basejointList))
	def setBaseJointTorques(self,taus):
		n = 0;
		for j  in self._basejointList:
			pb.setJointMotorControlMultiDof(self._robotId,j, controlMode=pb.TORQUE_CONTROL,force=[taus[n]])
			n = n+1;	
	def printDebugLine(self,Tstart,Tend):	 
		pb.addUserDebugLine([Tstart[0,3],Tstart[1,3],Tstart[2,3]],[Tend[0,3],Tend[1,3],Tend[2,3]],lineColorRGB=[1,0,0])
 		
