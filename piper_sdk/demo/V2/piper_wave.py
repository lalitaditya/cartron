#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
import os
import math
from piper_sdk import *

def enable_piper(piper):
    print("Enabling Piper...")
    while not piper.EnablePiper():
        time.sleep(0.01)
    print("Enabled!")

def move_to_pos(piper, position, factor):
    joint_0 = round(position[0]*factor)
    joint_1 = round(position[1]*factor)
    joint_2 = round(position[2]*factor)
    joint_3 = round(position[3]*factor)
    joint_4 = round(position[4]*factor)
    joint_5 = round(position[5]*factor)
    # Gripper
    joint_6 = round(position[6]*1000*1000)
    
    piper.MotionCtrl_2(0x01, 0x01, 30, 0x00) # Speed 30
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    time.sleep(0.02)

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    
    enable_piper(piper)
    
    factor = 57295.7795
    
    # 1. Start at Zero
    print("Moving to Start Position...")
    start_pos = [0,0,0,0,0,0,0]
    for _ in range(50): # Send for a bit to ensure it moves
        move_to_pos(piper, start_pos, factor)
    
    # 2. Move to Wave Pose (Lift arm)
    # Based on existing demo pos: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.08]
    # Let's lift J3/J4 a bit more to "up" the hand
    print("Moving to Wave Position...")
    wave_base = [0.0, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0] 
    # J2=0.2 (Shoulder up), J4=0.5 (Wrist Pitch up?)
    
    # Interpolate to smooth motion (simple step)
    steps = 100
    for i in range(steps):
        curr = [s + (w - s) * i / steps for s, w in zip(start_pos, wave_base)]
        move_to_pos(piper, curr, factor)
        
    print("Ready to wave!")
    time.sleep(0.5)
    
    # 3. Say Hi
    print("Saying HI...")
    os.system("spd-say 'Hello there! I am Piper.'")
    
    # 4. Wave Motion (Use Joint 5 - Wrist Pan instead of J6 - Roll)
    # This creates a "side-to-side" wave if the arm is upright
    print("Waving with wrist...")
    wave_amp = 0.5 
    center_j5 = 0.0
    center_j6 = 0.0
    
    # Smooth sine wave for 3 cycles
    total_cycles = 4
    points_per_cycle = 40
    for i in range(total_cycles * points_per_cycle):
        t = (i / points_per_cycle) * 2 * math.pi
        
        # Primary Wave: J5 (Index 4)
        offset_j5 = math.sin(t) * wave_amp
        
        # Secondary Motion: J6 (Index 5) slightly, for flair
        offset_j6 = math.cos(t) * (wave_amp * 0.3)
        
        wave_pos = list(wave_base)
        wave_pos[4] = center_j5 + offset_j5
        wave_pos[5] = center_j6 + offset_j6
        
        move_to_pos(piper, wave_pos, factor)
            
    print("Done waving.")
    
    # 5. Return to Zero
    print("Returning to rest...")
    # Get current pos (from end of loop)
    current_pos = list(wave_base)
    current_pos[4] = center_j5 
    current_pos[5] = center_j6
    
    steps = 100
    # Interpolate Back
    for i in range(steps):
        # Linear interp from wave_base to start_pos
        curr = [w + (s - w) * (i / steps) for s, w in zip(current_pos, start_pos)]
        move_to_pos(piper, curr, factor)
        
    for _ in range(20):
         move_to_pos(piper, start_pos, factor)

