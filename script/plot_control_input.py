#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import datetime

# Initialize ROS node
rospy.init_node('cmd_vel_plotter', anonymous=True)

# Deques store the time and velocity data
times = deque(maxlen=50)  # Adjust size as needed
linear_velocities = deque(maxlen=50)
angular_velocities = deque(maxlen=50)

# Initial range for y-axis
y_range_linear = [-1.0, 10.0]  # Modify initial y-axis range as needed
y_range_angular = [-10.0, 10.0]

def cmd_vel_callback(msg):
    now = datetime.datetime.now()
    times.append(now)
    linear_velocities.append(msg.linear.x)
    angular_velocities.append(msg.angular.z)
    # Update the y-axis range if the values exceed the current range
    y_range_linear[0] = min(y_range_linear[0], msg.linear.x)
    y_range_linear[1] = max(y_range_linear[1], msg.linear.x)
    y_range_angular[0] = min(y_range_angular[0], msg.angular.z)
    y_range_angular[1] = max(y_range_angular[1], msg.angular.z)

# Subscribe to the cmd_vel topic
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

# Set up the plot
fig, axs = plt.subplots(2)
axs[0].set_title('Linear Velocity')
axs[1].set_title('Angular Velocity')

def animate(i):
    axs[0].cla()
    axs[1].cla()
    axs[0].plot(times, linear_velocities, label='Linear Velocity (m/s)')
    axs[1].plot(times, angular_velocities, label='Angular Velocity (rad/s)')
    axs[0].legend(loc='upper left')
    axs[1].legend(loc='upper left')
    axs[0].set_ylim(y_range_linear)
    axs[1].set_ylim(y_range_angular)

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()

# Keep the node running
rospy.spin()

