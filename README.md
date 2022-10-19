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

# Writing an action server and client
Prerequisites
You will require both the Fibonacci.action interface from the previous tutorial, "Creating an action," and the action tutorials interfaces package.

## Tasks:
1. Writing an action server
You should create a new file in your home directory called fibonacci action server.py and add the following code to it:
```
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
 ```
Now lets run our server:
```
python3 fibonacci_action_server.py
```
![Screenshot 2022-10-13 132708](https://user-images.githubusercontent.com/58104378/195501507-f68f1194-1c30-4b42-9eb4-e8fddac371e0.png)

We can communicate a goal via the command line interface to another terminal:
```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

![Screenshot 2022-10-13 132830](https://user-images.githubusercontent.com/58104378/195501705-6a5f4600-5c65-4daa-93d9-561a13b9ea21.png)

You should see the logged message "Executing goal..." followed by a notice that the goal state was not established in the terminal that is running the action server. The aborted state is assumed by default if the goal handle state is not set in the execute callback.

The succeed() method on the goal handle can be used to show that the goal was successful:

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        goal_handle.succeed()

        result = Fibonacci.Result()
        return result
You should see the goal completed with the status SUCCEED if you restart the action server and send another goal at this point.

![1](https://user-images.githubusercontent.com/58104378/195502784-454a64fd-026c-4db7-90cb-82dd48dd9149.png)
![2](https://user-images.githubusercontent.com/58104378/195502806-a9546d3b-13a8-4a08-bc65-c171445e2ce9.png)

Let's now make sure that our target execution computes and returns the specified Fibonacci sequence:

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        sequence = [0, 1]



        for i in range(1, goal_handle.request.order):

            sequence.append(sequence[i] + sequence[i-1])


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = sequence

        return result
We compute the sequence, assign it to the result message field, and then proceed to the return.

Send another goal and restart the action server. The aim should be completed with the expected results in order.

![1](https://user-images.githubusercontent.com/58104378/195503999-3eee5da8-855e-4910-8f6c-c6a398dabd1b.png)
![2](https://user-images.githubusercontent.com/58104378/195504016-1dcb1c7d-33f0-4d5e-8a63-f694858b753a.png)

 1.2 Publishing feedback
The sequence variable will be swapped out, and the sequence will now be stored in a feedback message. We publish the feedback message and then fall asleep after each update of the feedback message in the for-loop for impact:

```
import time


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        feedback_msg = Fibonacci.Feedback()

        feedback_msg.partial_sequence = [0, 1]


        for i in range(1, goal_handle.request.order):

            feedback_msg.partial_sequence.append(

                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = feedback_msg.partial_sequence

        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
 ```
 
Utilizing the command line tool with the --feedback option after restarting the action server, we can verify that feedback has now been published:
```
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```
![3](https://user-images.githubusercontent.com/58104378/195504876-9bd3445c-5f65-4b82-858d-8b2e13838ed7.png)
![4](https://user-images.githubusercontent.com/58104378/195504903-aa6372a7-73eb-4f33-be5c-6b401d534992.png)

 2. Writing an action client
We'll limit the action client to just one file as well. Then, open a new file and name it fibonacci action client.py. Add the following boilerplate code to the new file:
```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
```

Let's test our action client by first launching the earlier-built action server:
```
python3 fibonacci_action_server.py
```
Run the action client in an other terminal.

```
python3 fibonacci_action_client.py
```
As the action server completes the goal, the following messages should be printed:
```
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
# etc.
```
![5](https://user-images.githubusercontent.com/58104378/195506429-572652e0-9525-4b8a-9f39-7c00515b0d4f.png)
The action client should begin and complete as soon as possible. We currently have a working action client, but we receive no feedback or results.

 2.1 Getting a result
We must first obtain a goal handle for the goal that we sent. The result can then be requested using the goal handle.

The full code for this example is provided here:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```
Go ahead and attempt to run our Fibonacci action client while an action server is running in a separate terminal!
```
python3 fibonacci_action_client.py
```
![6](https://user-images.githubusercontent.com/58104378/195507078-14f35a6e-f937-434a-9af7-62a637c55a0c.png)

You should be able to see the goal being accepted and the outcome in the logs.

 2.2 Getting feedback
We can send goals to our action client. Nice! However, it would be wonderful to hear some input regarding the goals we transmit from the action server.

Here is the whole code for this illustration:
```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```
Everything is ready for us. Your screen should display feedback if we run our action client.
```
python3 fibonacci_action_client.py
```
![7](https://user-images.githubusercontent.com/58104378/195507534-d84206f0-29f5-4cd6-a97b-d13026a7b597.png)

# 3.Composing multiple nodes in a single process
## 3.1. To discover avilable components

In order to check the available components in the workspace, we run the following commands.
```
ros2 component types
```
The terminal returns all the available components:

![1](https://user-images.githubusercontent.com/58104378/196777132-cb8dd7be-9725-4107-8a4b-67128efedd9e.png)

## 3.2. Run-time composition using ROS services with a publisher and subscriber

At first, lets start the component container in one terminal :
```
ros2 run rclcpp_components component_container
```
To verify that the container is running via ros2 command line tools, we run following command in the second terminal which will show a name of the component as an output.
```
ros2 component list
```
 & in the second terminal, we load the talker component:

```
ros2 component load /ComponentManager composition composition::Talker
```
This command will return the unique ID of the loaded component as well as the name of the node:

After this, we run following code in the second terminal in order to load the listener component:
```
ros2 component load /ComponentManager composition composition::Listener
```
![2](https://user-images.githubusercontent.com/58104378/196778749-54b3b501-9734-41dc-a5ed-869911f7341e.png)
![3](https://user-images.githubusercontent.com/58104378/196778764-b9d34d8e-a31a-4544-aacd-afd90bc24d14.png)
![4](https://user-images.githubusercontent.com/58104378/196778779-c646593a-6fc9-43a5-969c-bb23f2884e2d.png)


Finally we can run the ros2 command line utility to inspect the state of the container:
```
ros2 component list
```
We can see the result as follows:

```
/ComponentManager
   1  /talker
   2  /listener
```
## 3.3 Run-time composition using ROS services with a server and client

It is very similar steps to what we did using talker and listener.

In the first terminal, we run:
```
ros2 run rclcpp_components component_container
```
and after that, in the second terminal, we run following commands to see server and client source code:
```
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```
![5](https://user-images.githubusercontent.com/58104378/196779881-8bd267e4-b790-49b3-9199-ee7e06cf4997.png)

![6](https://user-images.githubusercontent.com/58104378/196779908-4c89c056-7f38-45ab-ab74-2deace126d56.png)

## 3.4. Compile-time composition using ROS services

By using this demonstration, it shows that the same shared libraries can be reused to compile a single executable running multiple components.

The executable contains all four components from above : talker, listener, server, and client.

In one terminal,
```
ros2 run composition manual_composition
```

![7](https://user-images.githubusercontent.com/58104378/196780394-106a76b3-4f0e-415c-99b9-fb73a42b994f.png)

## 3.5. Run-time composition using dlopen

This demonstration shows an alternative to run-time composition by creating a generic container process an explicity passing the libraries to load without using ROS interfaces. The process will open each library and create one instance of each "rclcpp::Node" class in the library source code.
```
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```
![8](https://user-images.githubusercontent.com/58104378/196781052-3f184319-57c3-42d3-8557-cbe778f4d14b.png)

## 3.6. Composition using launch actions

While the command line tools are helpful for troubleshooting and diagnosing component setups, starting a group of components at once is frequently more practical. We can make use of ros2 launch's functionality to automate this process.
```
ros2 launch composition composition_demo.launch.py
```

![9](https://user-images.githubusercontent.com/58104378/196781549-edb27897-735d-40bd-9fb3-1ea07ac36261.png)

# 4.Creating a launch file
In order to create a launch file, we use the rqt_graph and turtlesim packages which we have already installed previously.

## 4.1. Setup

We need to create a new directory to store the launch files:
```
mkdir launch
```
## 4.3. ros2 launch

In order to run the launch file created, we enter into the earlier created directory and run the following commands:
```
cd launch
ros2 launch turtlesim_mimic_launch.py
```
![10](https://user-images.githubusercontent.com/58104378/196783569-3ca3489c-5014-4be1-969c-ad75e3b901ef.png)

two turtlesim windows are opened as shown below:
![a](https://user-images.githubusercontent.com/58104378/196783854-f8f057a6-7619-47f1-b97c-db0c2420fc05.png)

In order to see the sytem in action, we open a new terminal and run the ros2 topic pub command on /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving.

```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

You will see that those two turtle will start moving/spinning following the same path:
![b](https://user-images.githubusercontent.com/58104378/196784528-878e9b56-9622-4258-8188-cca426dd849e.png)

## 4.4. Introspect the system with rqt_graph

Open the new terminal without closing the system and we run rqt_graph.
```
rqt_graph
```

![c](https://user-images.githubusercontent.com/58104378/196785349-10b5df12-a6a7-45db-8119-db9285f78784.png)

# 5.Integrating launch files into ROS2 packages
## 5.1. Creating a package

Firstly, we create a workspace for the package:
```
mkdir -p launch_ws/src
cd launch_ws/src
```
& create a python package:

```
ros2 pkg create py_launch_example --build-type ament_python
```
## 5.2. Creating the structure to hold launch files 
In order to colcon to launch files, we need to inform Python's setup tools of our launch files using the data_files parameter of setup.

Inside, setup.py file, we input the following codes.
```
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```
## 5.3. Writing the launch file

Inside the launch directory, we create a new launch file named my_script_launch.py. Here, the launch file should define the generate_launch_description() function which returns a launch.LaunchDescription() to be used by the ros2 launch` verb.
```
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
  ```
 ## 5.4. Building and running the launch file

We go to top-level of the workspace and build the file using:

```
colcon build
```
Once the build is successful, we should be able to run the launch file as follows:
```
ros2 launch py_launch_example my_script_launch.py
```
![d](https://user-images.githubusercontent.com/58104378/196807513-1e43748a-1bbc-49ce-83a6-7cd8a0019c3f.png)

# 6.Using Substitutions
## 6.1. Creating and Setting up the package

We create a new package of build_type ament_python named launch_tutorial :
```
ros2 pkg create launch_tutorial --build-type ament_python
```
and inside of that package, we create a directory called launch.
```
mkdir launch_tutorial/launch
```
After that, we edit the setup.py file and add in changes so that launch file will be installed successfully.
```
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```
## 6.2. Parent Launch File

After the above steps, we created a launch file named : example_main.launch.py in the launch folder of the launch_tutorial directory with the following codes in it:
```
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```
## 6.3. Substitutions example launch file

A new file is created in the same folder: example_substitutions.launch.py
```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```
## 6.4. Building the package

We run the build command in the root of the workspace.
```
colcon build
```
## 6.5. Launching Example

Now we are able to run the example_main.launch.py file using the ros2 launch command.
```
ros2 launch launch_tutorial example_main.launch.py
```
![e](https://user-images.githubusercontent.com/58104378/196815243-70cb4d75-3083-49a9-a06d-07de7ad188c1.png)

A turtlesim node is started with a blue background. After that second turtle is spawned and the background color is changed to purple and pink respectively.

# 7.Using Event Handlers
## 7.1. Event handler example launch file

We created a new file named: example_event_handlers.launch.py in the same directory.. i.e. inside launch folder of launch_tutorial package.
```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])

```
## 7.2. Building and Running the Command

After adding the file, we go back to the root of the workspace and run the build command there.
```
colcon build
```
After building, it is important to source the package and run the following codes for the output:

```
ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```
![f](https://user-images.githubusercontent.com/58104378/196818216-388e8451-fa7e-4d20-b44c-f3e3cb35573f.png)
It spawns the second turtle and starts a turtlesim node with a blue backdrop. The background color then changes to pink and then purple. When the turtlesim window closes, the launch file also shuts down.


