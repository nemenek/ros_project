import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.msg import Color
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, SetPen, Kill, Spawn
PI = 3.141592653589793
Tolerance = 0.0001

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
        if forward:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        rate = rospy.Rate(100)

        self.twist_pub.publish(vel_msg)
        distance_traveled = 0.0
        
        
        while (abs(distance_traveled - distance) > Tolerance) and not(rospy.is_shutdown()):
            distance_traveled = math.sqrt((pow((self.pose.x - starting_pose.x), 2) + pow((self.pose.y - starting_pose.y), 2)))
            if distance_traveled > distance and forward:
                forward = not(forward)
                vel_msg.linear.x = -vel_msg.linear.x/1.5
                self.set_color(68, 86, 255, 0, 'Name')
            
            if (distance_traveled < distance and not(forward)):
                forward = not(forward)
                vel_msg.linear.x = -vel_msg.linear.x/1.5
                tc.set_color(0, 255, 0, 0, 'Name')
            
            self.twist_pub.publish(vel_msg)
            print('Distance traveled: ', str(distance_traveled))
            print('Goal distance: ', str(distance))
            rate.sleep()


        # Set velocity to 0
        vel_msg.linear.x = 0
        self.twist_pub.publish(vel_msg)
        print('Final distance traveled: ' + str(distance_traveled))


    # Turn using time, and angular velocity omega
    def turn(self, omega, angle, clockwise):
        angular_speed = 2*PI*omega/360
        relative_angle = 2*PI*angle/360

        vel_msg = Twist()
        if clockwise:
            goal_angle = self.pose.theta - relative_angle
            vel_msg.angular.z = - abs(angular_speed)
        else:
            goal_angle = self.pose.theta + relative_angle
            vel_msg.angular.z =  abs(angular_speed)
        if goal_angle < 0:
            goal_angle = math.radians(360-abs(math.degrees(goal_angle)))
            
        if math.degrees(goal_angle) < 355 and math.degrees(goal_angle) > 350:
            goal_angle = math.radians(360)
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        rate = rospy.Rate(10)

        self.twist_pub.publish(vel_msg)
        converted_angle = self.pose.theta
        
        while ((abs(math.degrees(converted_angle)-math.degrees(goal_angle)) > Tolerance) or ()) and not(rospy.is_shutdown()):
            if converted_angle < 0:
                converted_angle = math.radians(360-abs(math.degrees(converted_angle)))
            if (converted_angle > goal_angle and not(clockwise) and (goal_angle > converted_angle - math.radians(180))):
                clockwise = not(clockwise)
                vel_msg.angular.z = -self.pose.angular_velocity/1.5
            elif ((converted_angle < goal_angle) and (clockwise) and (converted_angle > goal_angle - math.radians(180))):
                clockwise = not(clockwise)
                vel_msg.angular.z = -self.pose.angular_velocity/1.5
            elif ((math.degrees(goal_angle) > 355) and math.degrees(converted_angle) < 10 and not(clockwise)):
                clockwise = not(clockwise)
                vel_msg.angular.z = -self.pose.angular_velocity/1.5
            elif ((math.degrees(goal_angle) < 5) and math.degrees(converted_angle) > 350 and clockwise):
                clockwise = not(clockwise)
                vel_msg.angular.z = -self.pose.angular_velocity/1.5
            self.twist_pub.publish(vel_msg)
            rate.sleep()
            converted_angle = self.pose.theta
            if converted_angle < 0:
                converted_angle = math.radians(360-abs(math.degrees(converted_angle)))
                
            print('Current angle: ' + str(math.degrees(converted_angle)))
            print('Goal angle: ' + str(math.degrees(goal_angle)))

        vel_msg.angular.z = 0
        self.twist_pub.publish(vel_msg)
        print('Final angle: ' + str(math.degrees(self.pose.theta)))


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
            
    def tree_rec(self, length, n):
        if length < (length / n):
            return
        self.go_straight(length* 0.5, length / n, True)
        self.turn(100, 45, False)
        self.tree_rec(length * 0.5, length / n)
        self.turn(100, 20, False)
        self.tree_rec(length * 0.5, length / n)
        self.turn(100, 75, True)
        self.tree_rec(length * 0.5, length / n)
        self.turn(100, 20, True)
        self.tree_rec(length * 0.5, length / n)
        self.turn(100, 30, False)
        #self.turn(100, 180, False)
        self.go_straight(3, length, False)
        return

if __name__ == '__main__':
    tc = TurtlesimProject()
    tc.reset_sim()
    tc.kill_turtle('turtle1')
    tc.spawn_turtle(3.5, 1, 0, 'Name')
    tc.set_color(0, 255, 0, 0, 'Name')
    rospy.sleep(1)
    tc.tree_rec(0.5, 4)
    #tc.test_rec(0.15, 4)
    tc.kill_turtle('Name')


