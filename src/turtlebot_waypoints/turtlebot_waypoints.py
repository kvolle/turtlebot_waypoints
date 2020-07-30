import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

# Define constants
TWO_PI = 6.28318
PI = 3.14159

# Sign function for use in the controller
def sgn(a):
    if (a>0):
        return 1.
    else:
        return -1.

def wrap(a):
    while(a > PI):
        a -= TWO_PI
    while(a < -PI):
        a += TWO_PI
    return a

# Eventually move these constants to rosparams.
X_MIN = 0.
X_MAX = 2.
Y_MIN = 0.
Y_MAX = 2.
X_PTS = 5
Y_PTS = 5
N_HEADINGS = 8
N_SAMPLES = 10
TRANSLATION_THRESHOLD = 0.005
ROTATION_THRESHOLD = 0.00873

class TurtlebotWaypointController():
    def __init__(self):
        # Define the message and publisher
        self.vel_msg = Twist()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Attempt to subscribe to both the mocap and the gazebo odometry
        self.odom_sub = rospy.Subscriber('pose_stamped_nwu', PoseStamped ,self.odom_callback)
        self.pose_sub = rospy.Subscriber('odom', Odometry ,self.odom_callback)

        # Define controller parameters
        self.kd_p = 2.
        self.kh_p = 2.
        self.max_x = 0.6
        self.max_z = 0.5
        
        # Equally space points on each axis
        xspace = np.linspace(X_MIN, X_MAX, X_PTS)
        yspace = np.linspace(Y_MIN, Y_MAX, Y_PTS)
        yspace_neg = np.linspace(Y_MAX, Y_MIN, Y_PTS)

        # Convert points to meshgrid
        self.goal_x = np.empty((0))
        self.goal_y = np.empty((0))
        for x in xspace:
            self.goal_x = np.append(self.goal_x, x*np.ones_like(yspace))
            if (x % 2 == 0):
                self.goal_y = np.append(self.goal_y, yspace)
            else:
                self.goal_y = np.append(self.goal_y, yspace_neg)


        # Initialize pose, heading, goal, and indices
        self.x = -17.
        self.y = -17.
        self.goal_index = 0
        self.rotation_index = 0
        self.theta = 0.

        # Define integer flags for state
        self.at_grid_point = 0
        self.samples = 0
        self.stationary = 0

    def pose_handler(self,pose):
        # Get x, y and yaw
        self.x = pose.position.x
        self.y = pose.position.y
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion (orientation_list)

    def pose_stamped_callback(self,data):
        # Pass mocap data to pose_handler
        self.pose_handler(data.pose)

    def odom_callback(self, data):
        # Pass gazebo data to pose_handler
        self.pose_handler(data.pose.pose)
    def calc_vel(self):
        if (abs(self.heading_err) > ROTATION_THRESHOLD):
            # Turn and nothing else
            old_vel = self.vel_msg.angular.z
            self.vel_msg.linear.x *= 0.9
            self.vel_msg.angular.z = sgn(self.heading_err)*min(self.max_z, self.kh_p*abs(self.heading_err), max(0.03, 2*abs(old_vel)))
            
        elif ((self.dist_err > TRANSLATION_THRESHOLD) and(self.at_grid_point==0)):
            # Drive forward and nothing else
            old_vel = self.vel_msg.linear.x
            self.vel_msg.angular.z = sgn(self.heading_err)*min(self.max_z, self.kh_p*abs(self.heading_err), max(0.03, 2*abs(old_vel)))
            self.vel_msg.linear.x = min(self.max_x, self.kd_p*self.dist_err*np.sqrt(self.dist_err), max(0.01, 1.1*old_vel))


        else:
            self.vel_msg.linear.x = 0.
            if (self.at_grid_point==0):
                self.at_grid_point = 1
                self.commanded_yaw = np.linspace(self.theta, self.theta + TWO_PI, N_HEADINGS, endpoint=False)
            if (self.rotation_index < N_HEADINGS-1):
                if (self.samples < N_SAMPLES-1):
                    self.samples += 1
                    print("Goal: %d Heading: %d Samples: %d"%(self.goal_index, self.rotation_index, self.samples))
                else:
                    self.samples = 0
                    self.rotation_index += 1
            else:
                self.rotation_index = 0
                self.goal_index += 1
                self.at_grid_point = 0
            # Stop and stay stationary.
            # This needs measurement logic

    def get_error(self):
        # Calculate the distance and heading errors. By making the heading error be -pi to pi, the turns should always be the closest direction.
        # Currently using the true atan2, consider approximate if runtime is an issue
        x = self.goal_x[self.goal_index]
        y = self.goal_y[self.goal_index]
        self.dist_err = math.sqrt((x-self.x)*(x-self.x) + (y-self.y)*(y-self.y))
        #self.heading_err = approx_atan(y, self.goal_x) - self.theta
        #if (self.at_grid_point):
        #    rospy.loginfo("HALP")
        #    self.heading_err = self.commanded_yaw[self.rotation_index] - self.theta
        #else:
        #    self.heading_err = np.arctan2(y-self.y, x-self.x)# - self.theta
        if self.at_grid_point:
            self.heading_err = wrap(self.commanded_yaw[self.rotation_index]) - self.theta
        else:
            self.heading_err = wrap(np.arctan2(y-self.y, x-self.x)) - self.theta
        #if (self.heading_err > PI):
        #    self.heading_err = self.heading_err - TWO_PI
        #if (self.heading_err < -PI):
        #    self.heading_err = self.heading_err + TWO_PI

    def pub_cmd(self):
        # Publish the commanded velocity
        #rospy.loginfo("D: %.3f H: %.3f X: %.3f Z: %.3f G: %d R: %d" % (self.dist_err, self.heading_err, self.vel_msg.linear.x, self.vel_msg.angular.z, self.goal_index, self.rotation_index))
        self.pub.publish(self.vel_msg)

    def step(self):
        self.get_error()
        self.calc_vel()
        #if self.in_transit:
        #else if self.at_station:

        # else:
        self.pub_cmd()
