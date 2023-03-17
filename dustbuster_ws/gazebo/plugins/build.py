import os

# Execute 'cmake' command
cmake_cmd = "cmake -B build"
os.system(cmake_cmd)

# Change to 'build' directory
os.chdir("build")

# Execute 'make' command
make_cmd = "make"
os.system(make_cmd)

import shutil

# Set the paths for the source file and destination directory
src_file = 'libwheel_velocity_setter.so'
dst_dir = '../output'

# Copy the file to the destination directory
shutil.copy(src_file, dst_dir)
