from glob import glob
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


# timer.py

INIT = 0
WALK = 1
ROT = 2
ROTWALK = 3
ETAT = INIT
angle = 0.0
direction_rot = 0
ti = 0
speed = 0.05
step_size = 0.03
step_height = 0.03
max_speed=0.04
min_speed=0.002
PHASE = 0
import time

class TimerError(Exception):
    """A custom exception used to report errors in use of Timer class"""

class Timer:
    def __init__(self):
        self._start_time = None
        self.current_time = 0
        
    def start(self):
        """Start a new timer"""
        if self._start_time is not None:
            return None
        self._start_time = time.time()
    def pause(self):
        if self._start_time is None:
            print("Timer is not running. Use .start() to start it")
            return None
        self.current_time += time.time() - self._start_time
        self._start_time = None
    def get_current_time(self):
        self.current_time += time.time() - self._start_time
        return self.current_time
FIRST_MOVE = True

#create a stopwatch 

# import display
def set_leg_angles(alphas, leg_id, robot,params):
    # if leg_id!=1:
    #     return
    robot.legs[leg_id][0].goal_position = alphas[0]* 180 / math.pi
    robot.legs[leg_id][1].goal_position = alphas[1]* 180 / math.pi
    robot.legs[leg_id][2].goal_position = alphas[2]* 180 / math.pi

def get_leg_angles(alphas, leg_id, robot,params):
    # if leg_id!=1:
    #     return
    return([robot.legs[leg_id][0].goal_position,robot.legs[leg_id][1].goal_position,robot.legs[leg_id][2].goal_position])

def get_angles(robot, params):
    angles = []
    for i in range(1,7):
        angles.append(get_leg_angles([0,0,0], i, robot, params))
    print(angles)
    return angles

def walk(time, angle, leg_id, params):
    global step_size
    if(leg_id == 1 or leg_id == 3 or leg_id == 5):
        t = (2 * math.pi * time - math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(step_size * math.cos(t), 0.0, step_height * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-step_size + 2* step_size * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)
    else:
        t = (2 * math.pi * time + math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(step_size * math.cos(t), 0.0, step_height * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-step_size + 2 * step_size * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)


def rotate(time, direction_rot, leg_id, params):
    global step_size
    angle = 0
    if (leg_id == 1 or leg_id == 3 or leg_id == 5):
        if(leg_id == 3):
            t = (-direction_rot* 2 * math.pi * time - math.pi/2.0)%(2.0*math.pi)
        else:
            t = (direction_rot*2 * math.pi * time - math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(step_size * math.cos(t), 0.0, step_height * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-step_size + 2 * step_size * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)
    elif (leg_id == 2 or leg_id == 4 or leg_id == 6):
        if(leg_id == 6):
            t = (direction_rot*2 * math.pi * time + math.pi/2.0)%(2.0*math.pi)
        else:
            t = (-direction_rot*2 * math.pi * time + math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(step_size * math.cos(t), 0.0, step_height * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-step_size + 2 * step_size * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)


def robot_controller():
    global ETAT
    global angle
    global ti
    global direction_rot
    global speed
    pygame.event.pump()
    keys = pygame.key.get_pressed()
    move = False
    rot = False
    if(ETAT==INIT):
        if(ti > 0.5):
            ti = 0.0
        if keys[pygame.K_z]:
            angle = np.pi*5/4
            move = True
        if keys[pygame.K_s]:
            angle = np.pi/4
            move = True
        if keys[pygame.K_d]:
            angle = np.pi*3/4
            move = True
        if keys[pygame.K_q]:
            angle = 7*np.pi/4
            move = True
        if keys[pygame.K_q] and keys[pygame.K_s]:
            angle = 0
            move = True
        if keys[pygame.K_d] and keys[pygame.K_s]:
            angle = 2*np.pi/4
            move = True
        if keys[pygame.K_d] and keys[pygame.K_z]:
            angle = 4*np.pi/4
            move = True
        if keys[pygame.K_q] and keys[pygame.K_z]:
            angle = 6*np.pi/4
            move = True
        if keys[pygame.K_a]:
            direction_rot = -1
            rot = True
        if keys[pygame.K_e]:
            direction_rot = 1
            rot = True
        if(move and rot):
            ETAT = ROTWALK
        elif(move):
            ETAT = WALK
        elif(rot):
            ETAT = ROT
    if keys[pygame.K_UP]:
        if (speed < max_speed):
            speed += 0.001
    if keys[pygame.K_DOWN]:
        if (speed > min_speed):
            speed -= 0.001



def setPositionToRobot(x,y,z,robot,params,t):
    global ti
    global ETAT
    global speed
    global PHASE
    # Use your own IK function
    robot_controller()
    if(ETAT != INIT):
        ti += speed
    print(ti)
    if(ti > 0.5):
        ti = 0.0
        PHASE = 1 - PHASE
        ETAT = INIT
    if(ETAT == INIT):
        for leg_id in range(1, 7):
            alphas = np.array(walk(PHASE * 0.5, angle, leg_id, params))
            set_leg_angles(alphas, leg_id, robot, params)
    elif (ETAT == ROTWALK):
        for leg_id in range(1, 7):            
            alphasw = np.array(walk(ti + PHASE * 0.5, angle, leg_id, params))
            alphasr = np.array(rotate(ti + PHASE * 0.5, direction_rot, leg_id, params))
            alphas = (alphasr + alphasw)/2.0
            set_leg_angles(alphas, leg_id, robot, params)
    
    elif(ETAT == ROT):
        for leg_id in range(1, 7):
            alphas = rotate(ti + PHASE * 0.5, direction_rot, leg_id, params)
            set_leg_angles(alphas, leg_id, robot, params)

    elif(ETAT == WALK):
        for leg_id in range(1, 7):
            alphas = walk(ti + PHASE * 0.5, angle, leg_id, params)
            set_leg_angles(alphas, leg_id, robot, params)
    
        
    #state = sim.setJoints(targets)


def main():
    global ti
    first_move = True
    
    ports = pypot.dynamixel.get_available_ports()
    dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)
    # robotPath = "phantomx_description/urdf/phantomx.urdf"
    # sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
    robot = SimpleRobot(dxl_io)
    robot.init()
    time.sleep(0.1)
    robot.enable_torque()
    # Defining the shutdown function here so it has visibility over the robot variable
    def shutdown(signal_received, frame):
        # Handle any cleanup here
        print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
        robot.disable_torque()
        print("Done ticking. Exiting.")
        # sys.exit()
        # Brutal exit
        os._exit(1)

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

        for k, v in robot.legs.items():
            # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
            v[0].goal_position = 0
            v[1].goal_position = 0
            v[2].goal_position = 0

        robot.smooth_tick_read_and_write(3, verbose=False)
        setPositionToRobot(0, 0, 0, robot, 0, params)
        robot.smooth_tick_read_and_write(3, verbose=False)
        time.sleep(2)
        INIT = False
        timer=time.time()
        while(True):
            # if first_move:
            #     print("INITIAL POSITION")
            #     get_angles(robot, params)
            #     setPositionToRobot(0, 0, 0, robot, params,ti)
            #     get_angles(robot, params)
            #     print("FIRST MOVE")
            #     robot.smooth_tick_read_and_write(3, verbose=False)
            #     print("END FIRST MOVE")
            #     #time.sleep(3)
            #     first_move = False
            # else:
                print("NOT FIRST MOVE")
                setPositionToRobot(0, 0, 0, robot, params,ti)
                robot.tick_write(verbose=False)
                if(time.time() - timer > 0.025):
                    assert(False)
                while(time.time() - timer <= 0.025):
                    pass
                timer=time.time()
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
