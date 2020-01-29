try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import sys
import numpy as np

def runSim(startX, startY, startTheta, duration, controllerHandle):
    """Connects to V-REP teleports the robot to the given position and starts running.
    
    Position is specified by (startX, startY) with rotation startTheta.
    The robot moves for duration time steps.  On each 
    time step the function controllerHandle is called which produces (leftSpeed,
    rightSpeed) which is used to control the robot's motors.  After
    duration time steps, the simulation is stopped and the connection with
    V-REP is closed. The return value is a duration x 2 matrix containing the robot's
    (x, y) position for all simulated time steps."""
    
    # The start position has a z-dimension too, so add that.
    startPosition=[startX,startY,0.14]
    
    # plotting/analysis.  Each row represents the (x, y) position for each
    # time step.
    positionHistory = np.zeros((duration,2))

    try:

        vrep.simxFinish(-1) # just in case, close all opened connections
        clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
        if clientID != -1:
            print ('Connected to remote API server')
        
        vrep.simxSynchronous(clientID, True)
        
        # Get object handles for the robot and its parts
        __,robot=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking)
        __,leftMotor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
        __,rightMotor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)
        __,leftVisionSensor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftVisionSensor',vrep.simx_opmode_blocking)
        __,rightVisionSensor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightVisionSensor',vrep.simx_opmode_blocking)
        
        vrep.simxSetObjectPosition(clientID,robot,- 1,startPosition,vrep.simx_opmode_oneshot)
        vrep.simxSetObjectOrientation(clientID,robot,- 1,[0,0,startTheta],vrep.simx_opmode_oneshot)
        
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
        
        # initial call in buffer mode.
        vrep.simxGetVisionSensorImage(clientID,leftVisionSensor,1,vrep.simx_opmode_streaming)
        vrep.simxGetVisionSensorImage(clientID,rightVisionSensor,1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(clientID,robot,- 1,vrep.simx_opmode_streaming)
        
        # that the simulation step is finished (it is a blocking call).
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)
        for i in range(duration):
            __,__,leftImage=vrep.simxGetVisionSensorImage(clientID,leftVisionSensor,1,vrep.simx_opmode_buffer)
            __,__,rightImage=vrep.simxGetVisionSensorImage(clientID,rightVisionSensor,1,vrep.simx_opmode_buffer)

            leftIntensity = leftImage[0]
            rightIntensity = rightImage[0]

            # Very oddly, values that exceed 127 become negative.  Correcting for this.
            if leftIntensity < 0:
                leftIntensity = 255 + leftIntensity
            if rightIntensity < 0:
                rightIntensity = 255 + rightIntensity
            print('left: {}, right: {}'.format(leftIntensity,rightIntensity))

            leftSpeed,rightSpeed=controllerHandle(leftIntensity,rightIntensity)

            vrep.simxSetJointTargetVelocity(clientID,leftMotor,leftSpeed,vrep.simx_opmode_blocking)
            vrep.simxSetJointTargetVelocity(clientID,rightMotor,rightSpeed,vrep.simx_opmode_blocking)

            # Update positionHistory matrix.
            __,pos=vrep.simxGetObjectPosition(clientID,robot,- 1,vrep.simx_opmode_buffer)
            positionHistory[i,0] = pos[0]
            positionHistory[i,1] = pos[1]
            #print('x: {}, y: {}\n'.format(pos[0], pos[1]))

            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)

    finally:
        # Stop the sim and disconnect cleanly.
        vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(clientID,rightMotor,0,vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
        vrep.simxFinish(clientID)
        print ('Disconnected from V-REP')

    return positionHistory