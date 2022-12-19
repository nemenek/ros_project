#!/usr/bin/python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.msg import Color
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, SetPen, Kill, Spawn
PI = 3.141592653589793
Tol_angle = 0.01
Tol_dist = 0.0001

class TurtlesimProject:
    def __init__(self):
        rospy.init_node('turtlesim_controller', anonymous=True)

        self.twist_pub = rospy.Publisher('/Name/cmd_vel', Twist, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/Name/pose', Pose, self.cb_pose)


    # Callback for pose
    def cb_pose(self, msg):
        self.pose = msg


    # Move straight to a given distance, using time and velocity
    def go_straight(self, speed, distance, forward):
        tc.set_color(0, 255, 0, 0, 'Name')
        starting_pose = Pose()
        starting_pose.x = self.pose.x
        starting_pose.y = self.pose.y

        # Create and publish msg
        vel_msg = Twist()
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        rate = rospy.Rate(200)

        self.twist_pub.publish(vel_msg)
        distance_traveled = 0.0


        while (abs(distance_traveled - distance) > Tol_dist) and not(rospy.is_shutdown()):
            if forward:
                vel_msg.linear.x = speed
            else:
                vel_msg.linear.x = -speed

            distance_traveled = math.sqrt((pow((self.pose.x - starting_pose.x), 2) + pow((self.pose.y - starting_pose.y), 2)))
            p = abs(distance_traveled - distance) * 20
            vel_msg.linear.x = vel_msg.linear.x * p
            if distance_traveled > distance and forward:
                forward = not(forward)
                vel_msg.linear.x = - vel_msg.linear.x
                self.set_color(68, 86, 255, 0, 'Name')

            if (distance_traveled < distance and not(forward)):
                forward = not(forward)
                vel_msg.linear.x = - vel_msg.linear.x
                tc.set_color(0, 255, 0, 0, 'Name')

            self.twist_pub.publish(vel_msg)
            #print('Distance traveled: ', str(distance_traveled))
            #print('Goal distance: ', str(distance))
            rate.sleep()


        # Set velocity to 0
        vel_msg.linear.x = 0
        self.twist_pub.publish(vel_msg)
        #print('Final distance traveled: ' + str(distance_traveled))


    # Turn using time, and angular velocity omega
    def turn(self, omega, angle, clockwise):
        angular_speed = 2*PI*omega/360
        relative_angle = 2*PI*angle/360

        current_angle = math.degrees(self.pose.theta)

        if clockwise:
            goal_angle = current_angle - angle
        else:
            goal_angle = current_angle + angle

        if goal_angle < 0:
            goal_angle = goal_angle + 360

        if (goal_angle < 360 and goal_angle > 350) or (goal_angle < 0 and goal_angle > -10):
            goal_angle = 0

        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        rate = rospy.Rate(200)

        angle_diff = current_angle-goal_angle

        while not((abs(angle_diff) % 360) < Tol_angle ) \
               and not(rospy.is_shutdown()):

            if current_angle < 0:
                current_angle = 360-abs(current_angle)

            if goal_angle < 0:
                goal_angle = goal_angle + 360

            angle_diff = current_angle-goal_angle

            if clockwise:
                vel_msg.angular.z = - abs(angular_speed)
            else:
                vel_msg.angular.z =  abs(angular_speed)


            p = min(abs(angle_diff), 50) * 0.3

            vel_msg.angular.z = vel_msg.angular.z * p

            if (current_angle > goal_angle and not(clockwise) and (goal_angle > current_angle - 180)):
                clockwise = not(clockwise)
                vel_msg.angular.z = -vel_msg.angular.z
            elif ((current_angle < goal_angle) and (clockwise) and (current_angle > goal_angle - 180)):
                clockwise = not(clockwise)
                vel_msg.angular.z = -vel_msg.angular.z
            elif (goal_angle > 355) and (current_angle < 10) and not(clockwise):
                clockwise = not(clockwise)
                vel_msg.angular.z = -vel_msg.angular.z
            elif (goal_angle < 5) and (current_angle > 350) and clockwise:
                clockwise = not(clockwise)
                vel_msg.angular.z = -vel_msg.angular.z


            self.twist_pub.publish(vel_msg)
            rate.sleep()

            current_angle = math.degrees(self.pose.theta)

            if current_angle < 0:
                current_angle = 360-abs(current_angle)

            #print('Current angle: ' + str(current_angle))
            #print('Goal angle: ' + str(goal_angle))

        vel_msg.angular.z = 0
        self.twist_pub.publish(vel_msg)
        #print('Final angle: ' + str(math.degrees(self.pose.theta)))


    # Draw a square
    def draw_square(self, speed, omega, a):
        for i in range(4):
            self.go_straight(speed, a, True)
            self.turn(omega, 90, False)


    # Draw a polygon with N sides
    def draw_poly(self, speed, omega, N, a):
        angle = 360 / N
        for i in range(N):
            self.go_straight(speed, a, True)
            self.turn(omega, angle, False)

    def set_color(self, r, g, b, off, name):
        color_srvc = rospy.ServiceProxy('/'+ name + '/set_pen', SetPen)
        color_srvc(r, g, b, 2, off)
        rospy.wait_for_service('/'+name+'/set_pen')

    def reset_sim(self):
        clear_srvc = rospy.ServiceProxy('/reset', Empty)
        clear_srvc()
        rospy.wait_for_service('/reset')

    def kill_turtle(self, name):
        kill_srvc = rospy.ServiceProxy('/kill', Kill)
        kill_srvc(name)
        rospy.wait_for_service('/kill')

    def spawn_turtle(self, x, y, theta, name):
        spawn_srvc = rospy.ServiceProxy('/spawn', Spawn)
        spawn_srvc(x, y, theta, name)
        rospy.wait_for_service('/spawn')

    def test_rec(self, length, depth):
        if depth == 0:
            self.go_straight(3, length, True)
        else:
            self.test_rec(length, depth-1)
            self.turn(100, 60, False)
            self.test_rec(length, depth-1)
            self.turn(100, 120, True)
            self.test_rec(length, depth-1)
            self.turn(100, 60, False)
            self.test_rec(length, depth-1)

    def face_up(self):
        current_angle = self.pose.theta
        current_angle = math.degrees(current_angle)
        angles_to_turn = current_angle - 90
        clockwise = True

        if(angles_to_turn < 0):
            angles_to_turn = abs(angles_to_turn)
            clockwise = False

        if (angles_to_turn > 180):
            angles_to_turn = 360 - angles_to_turn
            clockwise = not(clockwise)

        self.turn(100, angles_to_turn, clockwise)

    def draw_A(self, size, speed):
        self.face_up()

        self.turn(100, 30, True)
        self.go_straight(speed, size*2, True)
        self.turn(100, 120, True)
        self.go_straight(speed, size*2, True)
        #self.set_color(0, 255, 0, 1, 'Name')
        self.turn(100, 180, True)
        self.go_straight(speed, size/1.25, True)
        self.turn(100, 60, False)
        self.set_color(0, 255, 0, 0, 'Name')
        self.go_straight(speed, size*1.15, True)

    def draw_M(self, size, speed):
        self.face_up()

        self.go_straight(speed, size*2, True)
        self.turn(200, 150, True)
        self.go_straight(speed, size*1.5, True)
        self.turn(200, 120, False)
        self.go_straight(speed, size*1.5, True)
        self.turn(200, 150, True)
        self.go_straight(speed, size*2, True)

    def draw_E(self, size, speed):
        self.face_up()

        self.turn(200, 90, True)
        self.go_straight(speed, size, True)
        self.turn(200, 180, True)
        self.go_straight(speed, size, True)
        self.turn(200, 90, True)
        self.go_straight(speed, size, True)
        self.turn(200, 90, True)
        self.go_straight(speed, size, True)
        self.turn(200, 180, True)
        self.go_straight(speed, size, True)
        self.turn(200, 90, True)
        self.go_straight(speed, size, True)
        self.turn(200, 90, True)
        self.go_straight(speed, size, True)


if __name__ == '__main__':
    tc = TurtlesimProject()
    tc.reset_sim()
    tc.kill_turtle('turtle1')
    tc.spawn_turtle(3.1, 1.5, 0, 'Name')
    tc.set_color(0, 255, 0, 0, 'Name')
    rospy.sleep(1)
    idx = 0
    #tc.draw_M(2, 5)
    #tc.draw_E(2, 5)
    #tc.turn(0, 120, False)
    #tc.turn(0, 120, False)
    while idx < 6:
        #tc.test_rec(0.15, 3)
        tc.test_rec(0.06, 4)
        tc.turn(100, 60, False)
        idx = idx + 1
    #tc.kill_turtle('Name')



