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

This is how you create an action.
