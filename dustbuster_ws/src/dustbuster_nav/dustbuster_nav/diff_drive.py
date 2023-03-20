#import rclpy
import subprocess

# Define the robot's geometry
d = 0.58  # Distance from center of mass to wheel axis
r_l = 0.216015  # Left wheel radius
r_r = 0.216015  # Right wheel radius

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

# Print the results
print(f"\nLeft wheel angular velocity: {w_l:.2f} rad/s")
print(f"Right wheel angular velocity: {w_r:.2f} rad/s")

def main():

    subprocess.call(["ros2", "run", "dustbuster_nav", "driver", "--leftwh", str(w_l), "--rightwh", str(w_r)])



if __name__ == "__main__":
    main()