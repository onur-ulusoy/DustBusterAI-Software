"""
diff_drive.py

This script takes mesaures of robot from robot_params.yaml and
calculates the left and right wheel velocities of center of mass of
a differential wheeled robot and calls driver.py to generate corresponding angular velocities.

Usage: ros2 run dustbuster_nav diff_drive 

Author: Onur Ulusoy
Date: 20.03.2023

This code is licensed under the MIT license.
"""

import subprocess
import yaml
import os

# Adjust working directory
script_dir = os.path.dirname(os.path.abspath(__file__))

os.chdir(script_dir + "/../../../../../../src/dustbuster_nav/dustbuster_nav")

# Read the values from the YAML file
with open('robot_params.yaml', 'r') as file:
    params = yaml.safe_load(file)
    d = params['d']
    r_l = params['r_l']
    r_r = params['r_r']

# Define the expected speed of the center of mass
v_c = 3  # Expected speed of center of mass along longitudinal axis

# Calculate the left and right wheel speeds
w = 2  # Angular velocity of the robot 
v_l = (v_c - d*w)
v_r = (v_c + d*w)

# Print the results
print(f"Left wheel speed: {v_l:.2f} m/s")
print(f"Right wheel speed: {v_r:.2f} m/s")

# Calculate the angular velocities of the wheels
w_l = v_l / r_l
w_r = v_r / r_r


def main():
    # Print the results
    print(f"\nLeft wheel angular velocity: {w_l:.2f} rad/s")
    print(f"Right wheel angular velocity: {w_r:.2f} rad/s") 

    subprocess.call(["ros2", "run", "dustbuster_nav", "driver", "--leftwh", str(w_l), "--rightwh", str(w_r)])



if __name__ == "__main__":
    main()