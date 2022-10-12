# 12194945_week6
# a.Managing Dependencies with rosdep
How to use rosdep tool?
Now that we have a fundamental grasp of rosdep, package.xml, and rosdistro, we are ready to use the program. The initialization of rosdep must be done first before using it for the first time using:

```
sudo rosdep init
rosdep update
```

![Screenshot 2022-10-12 135831](https://user-images.githubusercontent.com/58104378/195253899-5a123bc4-5d1c-4f53-b7e0-141831d463c6.png)

Finally, we may install dependencies using rosdep install. To install all dependencies, this is frequently run just once in a workspace with many packages. A call for that might appear as follows if the source-code-containing directory src was present in the workspace's root.
```
rosdep install --from-paths src -y --ignore-src
```
![Screenshot 2022-10-12 140104](https://user-images.githubusercontent.com/58104378/195254209-b80b543f-3657-4181-942d-7948e4163f2e.png)

# b.Creating an Action
## Prerequisites
Colcon and ROS 2 must be installed.

Establish a workspace and a package called "action tutorials interfaces":

(Remember to source first from your ROS 2 installation.)
```
mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces
```
![Screenshot 2022-10-12 140610](https://user-images.githubusercontent.com/58104378/195254824-19fdcbd0-d139-49d5-bde3-532e7b34d95b.png)

# Tasks:

# 1.Defining an action
Actions are described in .action files with the following format:

```
# Request
---
# Result
---
# Feedback
```
Create a directory called "action" in our ROS 2 package called "action tutorials interfaces":

```
cd action_tutorials_interfaces
mkdir action
```


![Screenshot 2022-10-12 141219](https://user-images.githubusercontent.com/58104378/195255583-20277888-47cd-4db4-a0f2-e05e4dbb1000.png)

Make a file called Fibonacci in the action directory. action that includes the following information:

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```
# 2.Building an action
Before we can use the new Fibonacci action type in our code, we must pass the definition to the rosidl code generation pipeline.

The following lines need be added to our CMakeLists.txt before the ament package() line in the action tutorials interfaces to achieve this:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```
We must also include the necessary dependencies in our package.xml file:

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

![Screenshot 2022-10-11 134908](https://user-images.githubusercontent.com/58104378/195256175-ca906d30-6bf5-4724-93cd-93152f53af5d.png)

![Screenshot 2022-10-11 135027](https://user-images.githubusercontent.com/58104378/195256254-5a216402-bf52-4cbb-af1c-952fd2548499.png)
The fact that action definitions contain additional metadata means that we must rely on action msgs (e.g. goal IDs).

The package containing the definition of the Fibonacci action should now be able to be built:

```
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
```
We are done!

Action types will often start with the term "action" and the package name. As a result, our new action will be referred to by its complete name, action tutorials interfaces/action/Fibonacci.

Using the command line tool, we can verify that our action was built successfully:

```
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```


![Screenshot 2022-10-11 135354](https://user-images.githubusercontent.com/58104378/195259649-eef285a7-a5e4-4be8-87d5-4b52c3ea11c4.png)
The definition of the Fibonacci action should appear on the screen.

