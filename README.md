# ROS2 Installation and Setup Guide

## Step 1: Install ROS2

1. Install build-essential gcc make perl dkms packages

   ```bash
   sudo apt install build-essential gcc make perl dkms
   ```

2. Follow the [ROS2 installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

3. Install python3-colcon-common-extensions package

   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

4. Downgrade the setuptools version

   ```bash
   pip3 install setuptools==58.2.0
   ```

## Step 2: Create a Workspace

1. Open a terminal and navigate to the desired location for your workspace.
2. Run the following command to create a new workspace:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   ```

   This will create a new workspace directory named `ros2_ws` with a `src` folder inside.

## Step 3: Setup the environment

1. Navigate to the `install` folder of your workspace:

   ```bash
   cd ~/ros2_ws/install
   source local_setup.bash
   ```

2. Copy these lines to the .bashrc

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
   ```

3. Refresh the terminal

## Step 4: Create your Python/Cpp packages

1. Create Python package

   ```bash
   ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
   ```

2. Create Cpp Package

   ```bash
   ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclpy
   ```

## Step 5: Build and Run Your Project

1. Go back to the root of your workspace:

   ```bash
   cd ~/ros2_ws
   ```

2. Build your project using the following command:

   ```bash
   colcon build
   ```

3. or Build specific package using the following command

   ```bash
   colcon build --packages-select 'your_package'
   ```

4. Run specific package using the following command

   ```bach
   cd install/{your_package}/lib/{your_package}/
   ./{package_name}
   ```

## Conclusion

Congratulations! You have successfully installed and set up ROS2, as well as built and run your project. You can now [provide any additional information or next steps for the user].
