#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
import os
import sys
import math
from piper_sdk import *

# Fix for "No backend available" on Windows: 
# Manually add the conda environment's Library\bin to the DLL search path 
# so pyusb can find libusb-1.0.dll
conda_path = r"C:\Users\aryan\miniconda3\envs\ros_env\Library\bin"
if os.path.exists(conda_path):
    if hasattr(os, 'add_dll_directory'):
        os.add_dll_directory(conda_path)
    else:
        os.environ['PATH'] = conda_path + os.pathsep + os.environ['PATH']

def enable_piper(piper):
    print("Enabling Piper...")
    while not piper.EnablePiper():
        time.sleep(0.1)
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
    time.sleep(0.005)

if __name__ == "__main__":
    # Windows Setup: can_name is the index (usually "0").
    # We use can_auto_init=False to prevent the SDK from trying to open "can0" via socketcan by default.
    # Then we manually create the CAN bus using the "gs_usb" bus type.
    print("Initializing Piper Arm (Windows / gs_usb)...")
    piper = C_PiperInterface_V2(can_name="0", can_auto_init=False, judge_flag=False)
    piper.CreateCanBus(can_name="0", bustype="gs_usb")
    piper.ConnectPort()
    
    enable_piper(piper)
    
    factor = 57295.7795
    
    # 1. Start at Zero
    print("Moving to Start Position...")
    start_pos = [0,0,0,0,0,0,0]
    for _ in range(50):
        move_to_pos(piper, start_pos, factor)
    
    # 2. Move to Wave Pose (Lift arm)
    print("Moving to Wave Position...")
    wave_base = [0.0, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0] 
    
    steps = 100
    for i in range(steps):
        curr = [s + (w - s) * i / steps for s, w in zip(start_pos, wave_base)]
        move_to_pos(piper, curr, factor)
        
    print("Ready to wave!")
    time.sleep(0.5)
    
    # 3. Say Hi (Windows Native Command)
    print("Saying HI...")
    speech_cmd = 'PowerShell -Command "Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak(\'Hello there! I am Piper.\')"'
    os.system(speech_cmd)
    
    # 4. Wave Motion (Use Joint 5 - Wrist Pan)
    print("Waving with wrist...")
    wave_amp = 0.5 
    center_j5 = 0.0
    center_j6 = 0.0
    
    total_cycles = 4
    points_per_cycle = 40
    for i in range(total_cycles * points_per_cycle):
        t = (i / points_per_cycle) * 2 * math.pi
        offset_j5 = math.sin(t) * wave_amp
        offset_j6 = math.cos(t) * (wave_amp * 0.3)
        
        wave_pos = list(wave_base)
        wave_pos[4] = center_j5 + offset_j5
        wave_pos[5] = center_j6 + offset_j6
        
        move_to_pos(piper, wave_pos, factor)
            
    print("Done waving.")
    
    # 5. Return to Zero
    print("Returning to rest...")
    current_pos = list(wave_base)
    current_pos[4] = center_j5 
    current_pos[5] = center_j6
    
    steps = 100
    for i in range(steps):
        curr = [w + (s - w) * (i / steps) for s, w in zip(current_pos, start_pos)]
        move_to_pos(piper, curr, factor)
        
    for _ in range(20):
         move_to_pos(piper, start_pos, factor)
    
    print("Task completed.")
