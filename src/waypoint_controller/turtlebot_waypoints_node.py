import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

class TurtlebotWaypointController():
    def __init__():
        self.vel_msg = Twist()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('odom', Odometry ,self.pose_callback

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

    def pose_callback(data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion (orientation_list)

    def calc_vel():
        if (abs(self.dist_err) > 0.005):
            old_vel = self.vel_msg.linear.x
            self.vel_msg.linear.x = min(self.max_x, self.k_p*np.cos(self.heading_err)*self.dist_err,max(0.05,1.1*old_vel))

        if (abs(self.heading_err) > 0.00873):
            old_vel = self.vel_msg.angular.z
            self.vel_msg.angular.z = sgn(self.heading_err)*min(self.max_z, self.kh_p*abs(self.heading_err)), max(0.01,1.1*old_vel))

    def get_error():
        x = self.goal_x[self.goal_index]
        y = self.goal_y[self.goal_index]
        self.dist_err = sqrt((x-self.x)*(x-self.x) + (y-self.y)*(y-self.y))
        #self.heading_err = approx_atan(y, self.goal_x) - self.theta
        self.heading_err = np.arctan2(y, self.goal_x) - self.theta

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
