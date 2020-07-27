import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

TWO_PI = 6.28318
PI = 3.14159

def sgn(a):
    if (a>0):
        return 1.
    else:
        return -1.

X_MIN = 0.
X_MAX = 2.
Y_MIN = 0.
Y_MAX = 2.
X_PTS = 5
Y_PTS = 5
N_HEADINGS = 8

class TurtlebotWaypointController():
    def __init__():
        self.vel_msg = Twist()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('pose_stamped_nwu', PoseStamped ,self.odom_callback)
        self.pose_sub = rospy.Subscriber('odom', Odometry ,self.odom_callback)

        self.kd_p = 2.
        self.kh_p = 2.
        self.max_x = 2.
        self.max_z = 0.5
        
        xspace = np.linspace(X_MIN, X_MAX, X_PTS-1)
        yspace = np.linspace(Y_MIN, Y_MAX, Y_PTS-1)

        
        self.goal_x, self.goal_y = np.meshgrid(xspace, yspace)
        self.goal_x = np.reshape(self.goal_x, (-1))
        self.goal_y = np.reshape(self.goal_y, (-1))

        self.x = self.goal_x[0]
        self.y = self.goal_y[0]
        self.goal_index = 0
        self.theta = 0.
        self.at_grid_point = 0

    def pose_handler(pose):
        self.x = pose.position.x
        self.y = pose.position.y
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientacdtion_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion (orientation_list)
    def pose_stamped_callback(data):
        self.pose_handler(data.pose)
    def odom_callback(data):
        self.pose_handler(data.pose.pose)
    def calc_vel():
        if (abs(self.dist_err) > 0.005):
            old_vel = self.vel_msg.linear.x
            self.vel_msg.linear.x = min(self.max_x, self.k_p*np.cos(self.heading_err)*self.dist_err,max(0.05,1.1*old_vel))
        else if (! self.at_grid_point):
            self.at_grid_point = 1
            self.commanded_yaw = np.linspace(self.theta, self.theta + TWO_PI, N_HEADINGS)
            self.rotation_index=0

        if (abs(self.heading_err) > 0.00873):
            old_vel = self.vel_msg.angular.z
            self.vel_msg.angular.z = sgn(self.heading_err)*min(self.max_z, self.kh_p*abs(self.heading_err)), max(0.01,1.1*old_vel))

    def get_error():
        x = self.goal_x[self.goal_index]
        y = self.goal_y[self.goal_index]
        self.dist_err = sqrt((x-self.x)*(x-self.x) + (y-self.y)*(y-self.y))
        #self.heading_err = approx_atan(y, self.goal_x) - self.theta
        if (self.at_grid_point):
            self.heading_err = self.commanded_yaw[self.rotation_index] - self.theta
        else:
            self.heading_err = np.arctan2(y, self.goal_x) - self.theta
        if (self.heading_err > PI):
            self.heading_err = self.heading_err - TWO_PI
        if (self.heading_err < -PI):
            self.heading_err = self.heading_err + TWO_PI

    def pub():
        self.pub.publish(self.vel_msg)

    def step():
        self.get_error()
        self.calc_vel()
        #if self.in_transit:
        #else if self.at_station:

        else:
        self.pub()
            

rospy.init_node('waypoint_controller')
r = rospy.Rate(10) # 10hz
turtle = TurtlebotWaypointController()
while not rospy.is_shutdown():
   turtle.step()
   r.sleep()
