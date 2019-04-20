
ros_node_configuration
---

The package contains a node that can flexibly create a list of devices at runtime based on a configuration file. It's an example application that showcases how a generic code along with a configuration file can be used to create what would otherwise be multiple instances of repetitive and hardcoded pieces of code. There is an accompanying [blog post](http://nlamprian.me/blog/software/ros/2019/04/14/ros-node-configuration/) that describes this concept.

Test
---

Install socat

```bash
sudo apt install socat
```

Create a directory for the serial ports

```bash
mkdir ~/dev
```

Create the serial ports

```bash
socat -d -d pty,raw,echo=0,link=/home/`whoami`/dev/ttyIMU pty,raw,echo=0,link=/home/`whoami`/dev/ttyIMU0 &
socat -d -d pty,raw,echo=0,link=/home/`whoami`/dev/ttySonarFront pty,raw,echo=0,link=/home/`whoami`/dev/ttySonarFront0 &
socat -d -d pty,raw,echo=0,link=/home/`whoami`/dev/ttySonarRear pty,raw,echo=0,link=/home/`whoami`/dev/ttySonarRear0 &
```

Write some data to the ports

```bash
while true; do
  echo "Hello from Imu: $RANDOM" > ~/dev/ttyIMU0;
  echo "Hello from SonarFront: $RANDOM" > ~/dev/ttySonarFront0;
  echo "Hello from SonarRear: $RANDOM" > ~/dev/ttySonarRear0;
  sleep 0.02;  # 50 Hz
done
```

Install dependencies

```bash
sudo apt install ros-${ROS_DISTRO}-serial
```

Download the package, compile and source the workspace, and finally start the launch file

```bash
roslaunch ros_node_configuration bringup.launch
```

The devices will publish data to a topic and diagnostic statuses.
