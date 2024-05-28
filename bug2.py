import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import tf_transformations as transform

class Bug2(Node):
    def __init__(self):


        super().__init__('bug2')
        self.odom_subscriber=self.create_subscription(Odometry,'/odom',self.location_callback,10)
        self.fl_sensor_sub=self.create_subscription(Range,'/fl_range_sensor',self.fl_sensor_callback, 10)
        self.fr_sensor_sub=self.create_subscription(Range, '/fr_range_sensor',self.fr_sensor_callback, 10)
        self.cmd_pub=self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub=self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.bug_algorithm_timer=self.create_timer(0.1, self.bug_algorithm_callback)
        self.goal_x=None
        self.goal_y=None

        self.current_x=0.0
        self.current_y=0.0


        self.current_theta=0.0
        self.fl_sensor_value=0.0
        self.fr_sensor_value=0.0
        self.cmd_vel_msg=Twist()
        self.state='find_direction'
        self.hit_point=None
        self.leave_point=None
        self.initial_position=None
        self.final_position=None
        self.direction_found=False





    def goal_callback(self,msg):
        self.goal_x=msg.pose.position.x
        self.goal_y=msg.pose.position.y
        self.final_position=msg.pose.position

    def fl_sensor_callback(self,msg):
        self.fl_sensor_value=msg.range / 0.65  
        if self.fl_sensor_value<0.0:
            self.fl_sensor_value=0.0


    def fr_sensor_callback(self,msg):
        self.fr_sensor_value=msg.range/0.65
        if self.fr_sensor_value<0.0:
            self.fr_sensor_value=0.0


    def location_callback(self,msg):
        self.current_x=msg.pose.pose.position.x
        self.current_y=msg.pose.pose.position.y
        self.current_position=msg.pose.pose.position
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )


        self.current_theta=transform.euler_from_quaternion(q)[2]
        if self.initial_position is None:
            self.initial_position=self.current_position



    def is_on_line(self):
        goal_vec=[self.goal_x-0.0,self.goal_y-0.0]
        pos_vec=[self.current_x-0.0,self.current_y- 0.0]
        cross_product= goal_vec[0] * pos_vec[1]- goal_vec[1]*pos_vec[0]
        return abs(cross_product)< 0.01



    def calculate_euclidean_distance(self,x1,y1,x2,y2):
        return math.sqrt((x1-x2)**2+(y1 - y2)**2)








    def bug_algorithm_callback(self):
        if self.goal_x is None or self.goal_y is None:
            self.cmd_vel_msg.linear.x =0.0
            self.cmd_vel_msg.angular.z= 0.0
            self.cmd_pub.publish(self.cmd_vel_msg)
            return



        if self.state =='find_direction':
            goal_angle= math.atan2(self.goal_y-self.current_y,self.goal_x-self.current_x)
            angle_diff =goal_angle - self.current_theta
            if angle_diff> math.pi:
                angle_diff-=2 *math.pi
            elif angle_diff<-math.pi:
                angle_diff+=2 *math.pi
            self.cmd_vel_msg.angular.z= 0.3*angle_diff
            self.cmd_vel_msg.linear.x= 0.0




            if abs(angle_diff) < 0.05:
                self.direction_found = True
                self.state ='move_to_goal'


        elif self.state == 'move_to_goal':
            if not self.direction_found:
                return
            if self.fl_sensor_value < 0.65 or self.fr_sensor_value < 0.65:
                self.hit_point=(self.current_x, self.current_y)
                self.state='follow_boundary'
                self.cmd_vel_msg.angular.z = 0.1  
                self.cmd_vel_msg.linear.x=0.0
            else:
                goal_angle= math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
                angle_diff =goal_angle - self.current_theta
                if angle_diff >math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff< -math.pi:
                    angle_diff += 2 * math.pi
                self.cmd_vel_msg.angular.z =0.3 * angle_diff  
                self.cmd_vel_msg.linear.x= 0.1  


        elif self.state == 'follow_boundary':
            if self.fl_sensor_value > 0.65 and self.fr_sensor_value > 0.65:
                self.state= 'move_to_goal'
                self.leave_point= (self.current_x, self.current_y)
                self.cmd_vel_msg.angular.z= 0.0
                self.cmd_vel_msg.linear.x= 0.1






        elif self.state == 'follow_boundary':
            if self.fl_sensor_value > 0.65 and self.fr_sensor_value > 0.65:
                self.state = 'move_to_goal'
                self.leave_point = (self.current_x, self.current_y)
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.1
            else:
                self.cmd_vel_msg.angular.z = 0.1
                self.cmd_vel_msg.linear.x = 0.0
                if self.is_on_line() and self.calculate_euclidean_distance(self.current_x, self.current_y, self.goal_x, self.goal_y) < 0.1:  # Promijenjen prag
                    self.state = 'move_to_goal'
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0



        self.cmd_pub.publish(self.cmd_vel_msg)








def main(args=None):


    rclpy.init(args=args)

    bug_node =Bug2()
    
    rclpy.spin(bug_node)
    
    bug_node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":


    main()
