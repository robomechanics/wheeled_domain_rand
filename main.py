import pybullet as p
import os
import tempfile
from generic_ind import generic_ind
import time
from wheeledSim.simController import simController
import yaml

if __name__=="__main__":
    template_fn = 'generic_ind.template'
    params_fn = 'generic_ind.yaml'
    """ generate xacro with parameters """
    params = yaml.safe_load(open(params_fn,'r'))
    xacroData = ''
    for key in params['xacro'].keys():
        xacroData=xacroData+'<xacro:property name="' + key + '" value="' + str(params['xacro'][key]) + '"/>'
    xacroData = open(template_fn,'r').read().replace('<!--Insert Properties Here-->',xacroData)
    xacroFile = tempfile.NamedTemporaryFile()
    xacroFile.write(bytes(xacroData,'utf-8'))
    """ generate urdf using xacro """
    urdfFile = tempfile.NamedTemporaryFile()
    os.system("xacro " + xacroFile.name + ">" + urdfFile.name)
    
    """ start pyBullet """
    physicsClient = p.connect(p.GUI)
    """ import robot """
    robot = generic_ind(urdfPath=urdfFile.name,physicsClientId=physicsClient,params=params['simulation'])
    """ add sim controller """
    sim = simController(robot,physicsClientId=physicsClient)
    for i in range(100):
        time.sleep(0.1)
        sim.controlLoopStep([1,1])
    p.disconnect()
