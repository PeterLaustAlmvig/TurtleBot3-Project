#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import sleep

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR


class Obstacle():
    def __init__(self,):
        collision_counter = 0
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle(collision_counter)

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        front_cone = []
        left_cone = []
        right_cone = []
        scan_filter = []

        samples = len(scan.ranges)  # The number of samples is defined in
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 3            # 1 <= samples_view <= samples

        #rospy.loginfo('scan ranges: %f', samples)
        #if samples_view > samples:
        #    samples_view = samples

        if samples_view is 3:
            for i in range(15):
                front_cone.append(scan.ranges[344+i])
            for i in range(15):
                front_cone.append(scan.ranges[i])
            for i in range(30):
                left_cone.append(scan.ranges[29+i])
                right_cone.append(scan.ranges[299+i])

            scan_filter.append(front_cone)
            scan_filter.append(left_cone)
            scan_filter.append(right_cone)

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2

            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            for j in range(30):
                if math.isnan(scan_filter[i][j]):
                    scan_filter[i][j] = 0.4
                elif scan_filter[i][j] == 0:
                    scan_filter[i][j] = 0.4
            """
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
            """
        return scan_filter

    def update_speed(self, x, z = 0):
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = z
        self._cmd_pub.publish(twist)
        rospy.loginfo("Moving at speed: %f", abs(x))
        return x

    def obstacle(self, collision_counter):
        #twist = Twist()
        curr_velocity = LINEAR_VEL
        loop_counter = 0

        while not rospy.is_shutdown():
            loop_counter += 1
            rospy.loginfo("Loop number: %d", loop_counter)

            #scan the cones
            lidar_distances = self.get_scan()
            rospy.loginfo('min front: %f', min(lidar_distances[0]))
            rospy.loginfo('min left: %f', min(lidar_distances[1]))
            rospy.loginfo('min right %f', min(lidar_distances[2]))
            min_distance = min(lidar_distances[0])


            #Check for collision
            #A collision from the front and the side at the same time is just counted as one collision
            if(min_distance <= 0.095):
                collision_counter += 1
                rospy.loginfo('Total collisions: %d', collision_counter)
            elif(min(lidar_distances[1]) <= 0.1 or min(lidar_distances[2]) <= 0.1):
                collision_counter += 1
                rospy.loginfo('Total collisions: %d', collision_counter)

            if 0 < min_distance < SAFE_STOP_DISTANCE + 0.10:

                if min(lidar_distances[1]) > STOP_DISTANCE + 0.10 and min(lidar_distances[1])>= min(lidar_distances[2]): #check left
                    rospy.loginfo('Turning right')
                    curr_velocity = self.update_speed(curr_velocity-0.02, z = 25)
                    sleep(0.25)

                elif min(lidar_distances[2]) > STOP_DISTANCE + 0.10 and min(lidar_distances[2])>= min(lidar_distances[1]): #check right, goes right if left< right dist.
                    rospy.loginfo('Turning left')
                    curr_velocity = self.update_speed(curr_velocity-0.02, z = -25)
                    sleep(0.25)

                else:
                    rospy.loginfo('*BEEP* *BEEP* *BEEP*')
                    curr_velocity = self.update_speed(-curr_velocity, 15)
                    for i in range(10):
                        sleep(0.25)

                    curr_velocity = self.update_speed(0, 0)

            else:
                curr_velocity = self.update_speed(LINEAR_VEL)
                #rospy.loginfo('Distance of the obstacle : %f', min_distance)


def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
