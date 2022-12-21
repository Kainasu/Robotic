import pypot.dynamixel
import time
import math
from utils import *
from onshape_to_robot.simulation import Simulation

import time

# from kinematics import *
import math
import sys
from signal import signal, SIGINT
import traceback
import os
import kinematics
import numpy as np
import pygame
pygame.display.set_mode((600,400))
pygame.init()


# import display
def set_leg_angles(alphas, leg_id, robot,params):
    # if leg_id!=1:
    #     return
    

        # for k, v in robot[leg_id].items():
        # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
    robot.legs[leg_id][0].goal_position = alphas[0]
    robot.legs[leg_id][1].goal_position = alphas[1]
    robot.legs[leg_id][2].goal_position = alphas[2]
        # v[0].goal_position = 0
        # v[1].goal_position = 0
        # v[2].goal_position = 0


def walk(time, angle, leg_id, params):
    if(leg_id == 1 or leg_id == 3 or leg_id == 5):
        t = (2 * math.pi * time - math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(0.02 * math.cos(t), 0.0, 0.03 * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-0.02 + 0.04 * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)
    else:
        t = (2 * math.pi * time + math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(0.02 * math.cos(t), 0.0, 0.03 * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-0.02 + 0.04 * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)


def rotate(time, direction_rot, leg_id, params):
    angle = 0
    if (leg_id == 1 or leg_id == 3 or leg_id == 5):
        if(leg_id == 3):
            t = (-direction_rot* 2 * math.pi * time - math.pi/2.0)%(2.0*math.pi)
        else:
            t = (direction_rot*2 * math.pi * time - math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(0.02 * math.cos(t), 0.0, 0.03 * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-0.02 + 0.04 * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)
    elif (leg_id == 2 or leg_id == 4 or leg_id == 6):
        if(leg_id == 6):
            t = (direction_rot*2 * math.pi * time + math.pi/2.0)%(2.0*math.pi)
        else:
            t = (-direction_rot*2 * math.pi * time + math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(0.02 * math.cos(t), 0.0, 0.03 * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-0.02 + 0.04 * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)

def robot_controller_walk():
    move = True
    angle = 0
    pygame.event.pump()
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT] and keys[pygame.K_DOWN]:
        angle = np.pi/4
    elif keys[pygame.K_RIGHT] and keys[pygame.K_DOWN]:
        angle = 3*np.pi/4
    elif keys[pygame.K_RIGHT] and keys[pygame.K_UP]:
        angle = 5*np.pi/4
    elif keys[pygame.K_LEFT] and keys[pygame.K_UP]:
        angle = 7*np.pi/4
    elif keys[pygame.K_UP]:
        angle = 3*np.pi/2
    elif keys[pygame.K_DOWN]:
        angle = np.pi/2
    elif keys[pygame.K_RIGHT]:
        angle = np.pi
    elif keys[pygame.K_LEFT]:
        angle = 0
    else :
        move = False
    return move, angle

def robot_controller_rot():
    rota = True
    direction_rot = 0
    pygame.event.pump()
    keys = pygame.key.get_pressed()
    if keys[pygame.K_a]:
        direction_rot = -1
    elif keys[pygame.K_e]:
        direction_rot = 1
    else:
        rota = False
    return rota, direction_rot

def setPositionToRobot(x,y,z,robot,params):
    # Use your own IK function
    move, angle = robot_controller_walk()
    rota, direction_rot = robot_controller_rot()

    if (rota and move):
        for leg_id in range(1, 7):            
            alphasw = np.array(walk(time.time(), angle, leg_id, params))
            alphasr = np.array(rotate(time.time(), direction_rot, leg_id, params))
            alphas = (alphasr + alphasw)/2.0
            set_leg_angles(alphas, leg_id, robot, params)
        
    
    elif(rota):
        for leg_id in range(1, 7):
            alphas = rotate(time.time(), direction_rot, leg_id, params)
            set_leg_angles(alphas, leg_id, robot, params)
        

    elif(move):
        for leg_id in range(1, 7):
            alphas = walk(time.time(), angle, leg_id, params)
            set_leg_angles(alphas, leg_id, robot, params)
        
        
        
    #state = sim.setJoints(targets)


def main():
    # ports = pypot.dynamixel.get_available_ports()
    # dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)
    robotPath = "phantomx_description/urdf/phantomx.urdf"
    sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
    robot = SimpleRobotSimulation(sim)
    robot.init()
    time.sleep(0.1)
    robot.enable_torque()
    # Defining the shutdown function here so it has visibility over the robot variable
    def shutdown(signal_received, frame):
        # Handle any cleanup here
        print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
        robot.disable_torque()
        print("Done ticking. Exiting.")
        sys.exit()
        # Brutal exit
        # os._exit(1)

    # Tell Python to run the shutdown() function when SIGINT is recieved
    signal(SIGINT, shutdown)
    try:

        params = Parameters(
            freq=50,
            speed=1,
            z=-60,
            travelDistancePerStep=80,
            lateralDistance=90,
            frontDistance=87,
            frontStart=32,
            method="minJerk",
        )

        print("Setting initial position")
        # for k, v in robot.legs.items():
        #     # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
        #     v[0].goal_position = math.pi/4.0
        #     v[1].goal_position = math.pi/4.0
        #     v[2].goal_position = math.pi/4.0

        # robot.smooth_tick_read_and_write(1, verbose=False)
        # for k, v in robot.legs.items():
        #     # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
        #     v[0].goal_position = 0
        #     v[1].goal_position = 0
        #     v[2].goal_position = 0
        while(True):
            setPositionToRobot(0, 0, 0, robot, params)
            robot.tick_read_and_write()
        print("Init position reached")
        time.sleep(2)
        print("Closing")
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


print("A new day dawns")
main()
print("Done !")
