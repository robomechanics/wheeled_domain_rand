import pybullet as p
import numpy as np
import os

class generic_ind:
    def __init__(self,urdfPath=None,physicsClientId=0,params={}):
        # Default parameters to use with Clifford
        self.params = {"maxThrottle":10,
                        "maxSteerAngle":0.5,
                        "susOffset":0.5,
                        "susDamping":1,
                        "susSpring":1,
                        "traction":1.5,
                        "massScale":1.0,
                        "tireMassScale":1.0}
        self.params.update(params)
        # set path of urdf if not included
        self.urdfPath = urdfPath
        # define which PyBullet simulation to use
        self.physicsClientId=physicsClientId
        # set up robot in simulation
        self.importBot()

    def importBot(self):
        self.robotID = p.loadURDF(self.urdfPath,physicsClientId=self.physicsClientId)#,useFixedBase=1)
        # define number of joints of robot
        nJoints = p.getNumJoints(self.robotID,physicsClientId=self.physicsClientId)
        self.buildModelDict()
        self.loosenModel()
        self.fix_wheel_roll()
        self.reset() 
        self.changeTraction()
        self.changeColor()

    def reset(self,pose=[[0,0,1.0],[0,0,0,1]]):
        pose=[[0,0,2.0],[0,0,0,1]]
        p.resetBasePositionAndOrientation(self.robotID, pose[0],pose[1],physicsClientId=self.physicsClientId)
        nJoints = p.getNumJoints(self.robotID,physicsClientId=self.physicsClientId)
        initialJointPositions = [(0,) for i in range(nJoints)]
        initialJointVelocities = [(0,) for i in range(nJoints)]
        for sus in ['fr','fl','br','bl']:
            initialJointPositions[self.jointNameToID[sus+'_body2link']] = (self.params['susOffset'],)
            initialJointPositions[self.jointNameToID[sus+'_roll_control']] = (-self.params['susOffset'],)
        p.resetJointStatesMultiDof(self.robotID,range(nJoints),initialJointPositions,initialJointVelocities,physicsClientId=self.physicsClientId)

    def buildModelDict(self):
        nJoints = p.getNumJoints(self.robotID,physicsClientId=self.physicsClientId)
        self.jointNameToID = {}
        self.linkNameToID = {}
        self.jointNames = []
        self.motorJointsIDs = []
        for i in range(nJoints):
            JointInfo = p.getJointInfo(self.robotID,i,physicsClientId=self.physicsClientId)
            self.jointNameToID[JointInfo[1].decode('UTF-8')] = JointInfo[0]
            self.linkNameToID[JointInfo[12].decode('UTF-8')] = JointInfo[0]
            self.jointNames.append(JointInfo[1].decode('UTF-8'))
            if 'drive' in self.jointNames[i]:
                self.motorJointsIDs.append(i)

    def loosenModel(self):
        nJoints = p.getNumJoints(self.robotID,physicsClientId=self.physicsClientId)
        for i in range(nJoints):
            if 'body2link' in self.jointNames[i]:
                p.setJointMotorControl2(bodyUniqueId=self.robotID,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=self.params['susOffset'],
                                        targetVelocity=0,
                                        positionGain=self.params['susSpring'],
                                        velocityGain=self.params['susDamping'],
                                        force=100,physicsClientId=self.physicsClientId)
            elif 'steer' in self.jointNames[i]:
                p.setJointMotorControl2(bodyUniqueId=self.robotID,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=0,
                                        positionGain=1,
                                        velocityGain=1,
                                        force=100,physicsClientId=self.physicsClientId)
            elif 'drive' in self.jointNames[i]:
                p.setJointMotorControl2(bodyUniqueId=self.robotID,
                                        jointIndex=i,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=0,
                                        positionGain=1,
                                        velocityGain=1,
                                        force=100,physicsClientId=self.physicsClientId)
            else:
                p.setJointMotorControl2(bodyUniqueId=self.robotID,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=0,
                                        positionGain=0,
                                        velocityGain=0,
                                        force=0,physicsClientId=self.physicsClientId)
    
    def changeColor(self,color=None):
        nJoints = p.getNumJoints(self.robotID,physicsClientId=self.physicsClientId)
        if color is None:
            color = [0.6,0.1,0.1,1]
        for i in range(-1,nJoints):
            p.changeVisualShape(self.robotID,i,rgbaColor=color,specularColor=color,physicsClientId=self.physicsClientId)
        wheels = ['fr_wheel','fl_wheel','br_wheel','bl_wheel']
        for wheel in wheels:
            p.changeVisualShape(self.robotID,self.linkNameToID[wheel],rgbaColor=[0.15,0.15,0.15,1],specularColor=[0.15,0.15,0.15,1],physicsClientId=self.physicsClientId)

    def fix_wheel_roll(self):
        for i in ['fr','fl','br','bl']:
            c = p.createConstraint(self.robotID,
                                   self.linkNameToID[i+'_sus_link'],
                                   self.robotID,
                                   self.linkNameToID[i+'_roll_link'],
                                   jointType=p.JOINT_GEAR,
                                   jointAxis=[1, 0, 0],
                                   parentFramePosition=[0, 0, 0],
                                   childFramePosition=[0, 0, 0],physicsClientId=self.physicsClientId)
            p.changeConstraint(c, gearRatio=1, maxForce=10000,physicsClientId=self.physicsClientId)

    def changeTraction(self,newTraction=None):
        if newTraction is None:
            newTraction = self.params['traction']
        for wheel in ['fr_wheel','fl_wheel','br_wheel','bl_wheel']:
            p.changeDynamics(self.robotID,self.linkNameToID[wheel],lateralFriction=newTraction,physicsClientId=self.physicsClientId)

    def updateSpringForce(self):
        return

    def drive(self,driveSpeed):
        maxForce = 10
        p.setJointMotorControlArray(self.robotID,self.motorJointsIDs,p.VELOCITY_CONTROL,
                                    targetVelocities=[driveSpeed*self.params["maxThrottle"]]*4,
                                    forces=[maxForce]*4,
                                    physicsClientId=self.physicsClientId)

    def steer(self,angle):
        maxForce = 10000
        p.setJointMotorControl2(bodyUniqueId=self.robotID, 
        jointIndex=self.jointNameToID['fr_steer'], 
        controlMode=p.POSITION_CONTROL,
        maxVelocity = 10,
        targetPosition = angle*self.params["maxSteerAngle"],
        force = maxForce,physicsClientId=self.physicsClientId)
        
        p.setJointMotorControl2(bodyUniqueId=self.robotID, 
        jointIndex=self.jointNameToID['fl_steer'], 
        controlMode=p.POSITION_CONTROL,
        maxVelocity = 10,
        targetPosition = angle*self.params["maxSteerAngle"],
        force = maxForce,physicsClientId=self.physicsClientId)
        
    def getBaseVelocity_body(self):
        gwb = p.getBasePositionAndOrientation(self.robotID,physicsClientId=self.physicsClientId)
        Rbw = p.invertTransform(gwb[0],gwb[1])[1]
        Vw = p.getBaseVelocity(self.robotID,physicsClientId=self.physicsClientId)
        v_b = p.multiplyTransforms([0,0,0],Rbw,Vw[0],[0,0,0,1])[0]
        w_b = p.multiplyTransforms([0,0,0],Rbw,Vw[1],[0,0,0,1])[0]
        return list(v_b)+list(w_b)

    def getPositionOrientation(self):
        pwb,Rwb = p.getBasePositionAndOrientation(self.robotID,physicsClientId=self.physicsClientId)
        return (pwb,Rwb)

    def measureJoints(self):
        jointStates = p.getJointStates(self.robotID,self.measuredJointIDs,physicsClientId=self.physicsClientId)
        positionReadings = [jointStates[i][0] for i in range(5)]
        velocityReadings = [jointState[1] for jointState in jointStates]
        measurements = positionReadings + velocityReadings
        return measurements
