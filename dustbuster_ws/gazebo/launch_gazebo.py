
"""
launch_gazebo.py

This script launches the Gazebo simulator along with the necessary plugins and models for the DustBusterAI robot.
It also starts a FastDDS discovery server in a separate thread.

Usage: python3 launch_gazebo.py

Author: Onur Ulusoy
Date: 17.03.2023

This code is licensed under the MIT license.

"""

import os
import subprocess
import threading

def start_fastdds_discovery():
    os.system('gnome-terminal --command "bash -c \'fastdds discovery --server-id 0; exec bash\'"')

try:
    # start the process in a separate thread
    fastdds_thread = threading.Thread(target=start_fastdds_discovery)
    fastdds_thread.start()

    # list the current directory
    subprocess.run(['ls'])

    # navigate to the output directory
    os.chdir('plugins/output')

    # print the current working directory for debugging
    subprocess.run(['pwd'])
    subprocess.run(['ls'])

    # source the set_plugin_path.bash file
    os.environ["GAZEBO_PLUGIN_PATH"] = os.getcwd()

    # navigate back to the parent directory
    os.chdir('../../')

    # start gazebo in a separate process
    gazebo_process = subprocess.Popen(['gazebo', '--verbose', 'room.world'])

    # wait for gazebo to terminate
    while gazebo_process.poll() is None:
        pass

    # gazebo has terminated, kill the fastdds discovery process
    os.system('pkill -f "fastdds discovery"')
    os.system('pkill -f "gzserver"')
    os.system('pkill -9 "gzserver"')


except KeyboardInterrupt:
    # Handle the case when user presses Ctrl+C
    print('\nExiting the program...')
    os.system('pkill -f "fastdds discovery"')
    os.system('pkill -f "gazebo"')
    os.system('pkill -9 "gzserver"')

