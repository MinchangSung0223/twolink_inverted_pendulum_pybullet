import pybullet as p
import numpy as np
import time
useMaximalCoordinates = False

p.connect(p.GUI)
pole = p.loadURDF("cartpole.urdf", [0, 0, 0], useMaximalCoordinates=useMaximalCoordinates)
zmp = p.loadURDF("zmp.urdf", [0, 0, 0], useMaximalCoordinates=useMaximalCoordinates)

for i in [0,2]:
  #disable default constraint-based motors
  p.setJointMotorControl2(pole, i, p.POSITION_CONTROL, targetPosition=0, force=0)
  #print("joint",i,"=",p.getJointInfo(pole2,i))

timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.1, 0.01)
desiredPosCartId = p.addUserDebugParameter("desiredPosCart", -10, 10, 2)
desiredVelCartId = p.addUserDebugParameter("desiredVelCart", -10, 10, 0)
kpCartId = p.addUserDebugParameter("kpCart", 0, 500, 1300)
kdCartId = p.addUserDebugParameter("kdCart", 0, 300, 150)
maxForceCartId = p.addUserDebugParameter("maxForceCart", 0, 5000, 1000)

textColor = [1, 1, 1]
shift = 0.05


desiredPosPoleId = p.addUserDebugParameter("desiredPosPole", -10, 10, 0)
desiredVelPoleId = p.addUserDebugParameter("desiredVelPole", -10, 10, 0)
kpPoleId = p.addUserDebugParameter("kpPole", 0, 500, 1200)
kdPoleId = p.addUserDebugParameter("kdPole", 0, 300, 100)
maxForcePoleId = p.addUserDebugParameter("maxForcePole", 0, 5000, 1000)


p.setGravity(0, 0, -10)

useRealTimeSim = False

p.setRealTimeSimulation(useRealTimeSim)

timeStep = 0.001
dt = timeStep
baseDof = 7
print(p.getJointInfo(pole,3))
p.enableJointForceTorqueSensor(pole, jointIndex=1)
prev_dq=np.array([0,0])

while p.isConnected():
  timeStep = p.readUserDebugParameter(timeStepId)
  p.setTimeStep(timeStep)

  desiredPosCart = p.readUserDebugParameter(desiredPosCartId)
  desiredVelCart = p.readUserDebugParameter(desiredVelCartId)
  kpCart = p.readUserDebugParameter(kpCartId)
  kdCart = p.readUserDebugParameter(kdCartId)
  maxForceCart = p.readUserDebugParameter(maxForceCartId)

  desiredPosPole = p.readUserDebugParameter(desiredPosPoleId)
  desiredVelPole = p.readUserDebugParameter(desiredVelPoleId)
  kpPole = p.readUserDebugParameter(kpPoleId)
  kdPole = p.readUserDebugParameter(kdPoleId)
  maxForcePole = p.readUserDebugParameter(maxForcePoleId)

  basePos, baseOrn = p.getBasePositionAndOrientation(pole)
  val=p.getJointState(pole, jointIndex=1)
  val2=p.getLinkState(pole, linkIndex=3)
  
  FT_sensor_value = val[2];
  com_value = val2[4];
  
  fx = FT_sensor_value[0];
  fy = FT_sensor_value[1];
  fz = FT_sensor_value[2];
  tx = FT_sensor_value[3];
  ty = FT_sensor_value[4];
  tz = FT_sensor_value[5];
  cz = com_value[2];
  
  cx = com_value[0];
  
  
  temp1=p.getJointState(pole, jointIndex=0)
  temp2=p.getJointState(pole, jointIndex=2)
  p.resetBasePositionAndOrientation(zmp,[-ty/(fz+0.0001)+temp1[0],0,0],[0,0,0,1])

  q = [temp1[0],temp2[0]]
  dq = [temp1[0],temp2[1]]
  ddq = [0,0]
  prev_dq = dq
  ID=  p.calculateInverseDynamics(pole,q,dq,ddq)
  taus = [-kpCart*temp1[0]-kdCart*temp1[1]+ID[0],0,-kpCart*temp2[0]-kdCart*temp2[1]];
  for j in [0, 2]:
    p.setJointMotorControlMultiDof(pole,
                                   j,
                                   controlMode=p.TORQUE_CONTROL,
                                   force=[taus[j]])
    #print(j,taus[j])


  if (not useRealTimeSim):
    p.stepSimulation()
    time.sleep(timeStep)
