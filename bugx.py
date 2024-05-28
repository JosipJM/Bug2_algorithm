import math
import numpy as np 
from time import sleep 
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class Bug2(Node):
    #Initialization
    def __init__(self):
        super().__init__('bug2')
        self.odom_subscriber=self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub=self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub=self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.target_sub=self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub= self.create_publisher(Twist, '/cmd_vel', 10)
        self.bug_algorithm_timer= self.create_timer(0.1, self.bug_algorithm_callback)
        self.linear_speed =0.1
        
        self.angular_speed =0.8
        
        self.theta_tolerance=2*(math.pi / 180)
        self.distance_tolerance= 0.1 
        self.distanceance_from_start_goal_tolerance= 0.1        
        
        
        
        
        self.angular_speed_theta_change= 0.062
        
        self.angular_speed_fast=0.785
        self.angular_speed_slow=0.35


        self.leave_point_to_hit_point_diff= 0.5

        self.max_distance_obs= 0.5 #udaljenost za trazenje prepreka
        self.distance_thresh_wf= 0.45 #udaljenost na kojoj se prati zid
        self.too_close_wall= 0.4 #je li je robot pre blizu yidu
        self.distance_to_wall= 0.5 #udaljenost kad je robot dosao do zida

        self.left_sensor= 0.0
        self.right_sensor= 0.0

        self.current_x= 0.0
        self.current_y= 0.0
        self.current_theta= 0.0

        self.goal_x= None
        self.goal_y= None

        self.robot_mode= "find goal"

        self.go_to_goal_state= "adjust heading"
        self.wall_following_state= "turn left"
        
        self.start_goal_line_calculated= False

        self.goal_line_slope= 0
        self.goal_line_intercept= 0
        self.start_x= 0
        self.goal_x= 0
        self.start_y= 0
        self.goal_y= 0

        self.rob_x_cord= 0
        self.rob_y_cord= 0

        self.last_robo_x= 0
        self.last_robo_y= 0

        self.rob_dist_to_goal= 0.0
        self.distanceance_to_goal_from_leave_point= 0.0
        


    def goal_callback(self, msg):
        self.goal_x= msg.pose.position.x
        self.goal_y= msg.pose.position.y

    def location_callback(self, msg):
        self.current_x= msg.pose.pose.position.x
        self.current_y= msg.pose.pose.position.y
        q= (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_theta= transform.euler_from_quaternion(q)[2] 

    def fl_sensor_callback(self, msg):
        self.left_sensor= msg.range
        

    def fr_sensor_callback(self, msg):
        self.right_sensor= msg.range
        


    def bug_algorithm_callback(self):
        if self.robot_mode=="Avoid obstacle":
            self.avoid_obstacles()
        
        if self.goal_x==False and self.goal_y==False:
            return
        
        self.bug2()

    def avoid_obstacles(self):


        msg= Twist()
        msg.linear.x= 0.0
        msg.linear.y= 0.0
        msg.linear.z= 0.0
        msg.angular.x= 0.0
        msg.angular.y= 0.0
        msg.angular.z= 0.0
         

        distance_obs= self.max_distance_obs

        if   self.left_sensor > distance_obs and self.right_sensor > distance_obs:
            msg.linear.x= self.linear_speed   

        elif self.left_sensor > distance_obs and self.right_sensor < distance_obs:
            msg.angular.z= self.angular_speed  

        elif self.left_sensor < distance_obs and self.right_sensor > distance_obs:
            msg.angular.z= -self.angular_speed 

        elif self.left_sensor < distance_obs and self.right_sensor < distance_obs:
            msg.angular.z= self.angular_speed  

        else:
            pass
             
        
        self.cmd_pub.publish(msg)




    def go_to_goal(self):



        msg= Twist()
        msg.linear.x= 0.0
        msg.linear.y= 0.0
        msg.linear.z= 0.0
        msg.angular.x= 0.0
        msg.angular.y= 0.0
        msg.angular.z= 0.0
    
        dist_to_wall= self.distance_to_wall
        too_close= self.too_close_wall

        if (self.left_sensor < dist_to_wall or self.right_sensor < too_close):
            self.robot_mode= "Follow the wall"

            
            self.rob_x_cord= self.current_x
            self.rob_y_cord= self.current_y

            self.rob_dist_to_goal= math.sqrt((self.goal_x-self.rob_x_cord)**2 +(self.goal_y-self.rob_y_cord)**2)  
            msg.linear.x= 0.0   
            msg.angular.z= self.angular_speed_fast +1 
                    
            self.cmd_pub.publish(msg)
                   
            return
      
        if (self.go_to_goal_state=="adjust heading"):


            desired_theta= math.atan2(self.goal_y-self.current_y,self.goal_x-self.current_x)
                   
            theta_error= desired_theta-self.current_theta
            
            if math.fabs(theta_error) > self.theta_tolerance:
             
                if theta_error > 0:          
                    msg.angular.z= self.angular_speed_theta_change   
                else:
                    msg.angular.z= -self.angular_speed_theta_change  
                 
                self.cmd_pub.publish(msg)
                 
            else:               
                self.go_to_goal_state= "Go straight"
                self.cmd_pub.publish(msg)        
                                   
        elif (self.go_to_goal_state=="Go straight"):
             
            position_error= math.sqrt((self.goal_x-self.current_x)**2 + (self.goal_y-self.current_y)**2)
                                              
            if position_error > self.distance_tolerance:
 
                msg.linear.x= self.linear_speed
                     
                self.cmd_pub.publish(msg)
                   
                desired_theta= math.atan2(self.goal_y-self.current_y,self.goal_x-self.current_x)
                 
                theta_error= desired_theta-self.current_theta      
         
                if math.fabs(theta_error) > self.theta_tolerance:
                     
                    self.go_to_goal_state= "adjust heading"
            else:           
                self.go_to_goal_state= "Goal achieved"

                self.robot_mode= "done"
                
                msg.linear.x= 0.0
                msg.angular.z= 0.0
                self.cmd_pub.publish(msg)


        elif (self.go_to_goal_state=="Goal achieved"):
         
            self.start_goal_line_calculated= False            
         
        else:

            pass

    
    def follow_wall(self):

        msg= Twist()
        msg.linear.x= 0.0
        msg.linear.y= 0.0
        msg.linear.z= 0.0
        msg.angular.x= 0.0
        msg.angular.y= 0.0
        msg.angular.z= 0.0   
        
        x_start_goal_line= self.current_x
        y_start_goal_line =(self.goal_line_slope *(x_start_goal_line)) +(self.goal_line_intercept)
        
        distanceance_from_start_to_goal= math.sqrt((x_start_goal_line-self.current_x)**2 + (y_start_goal_line-self.current_y)**2) 
                      
        if distanceance_from_start_to_goal < self.distanceance_from_start_goal_tolerance:


            self.last_robo_x= self.current_x
            self.last_robo_y= self.current_y

            self.distanceance_to_goal_from_leave_point= math.sqrt((self.goal_x-self.last_robo_x)**2 + (self.goal_y-self.last_robo_y)**2) 
            
            diff= self.rob_dist_to_goal-self.distanceance_to_goal_from_leave_point

            if diff > self.leave_point_to_hit_point_diff:


                self.robot_mode= "find goal"

                self.go_to_goal_state= "adjust heading"

                msg.linear.x= 0.0
                msg.angular.z= self.angular_speed_fast +2 
                    
                self.cmd_pub.publish(msg)

                return             
         
        dist_to_wall= self.distance_thresh_wf
        too_close= self.too_close_wall
         
        if self.left_sensor > dist_to_wall and self.right_sensor > dist_to_wall:
            #skretanje desno
            msg.linear.x= self.linear_speed
            msg.angular.z= -self.angular_speed_slow +0.05
             
        elif (self.left_sensor > dist_to_wall and self.right_sensor < dist_to_wall):
            if (self.right_sensor < self.too_close_wall):

                #skretanje lijevo
                msg.linear.x= self.linear_speed
                msg.angular.z= self.angular_speed_fast
            else:           

                #nastavi ravno
                msg.linear.x= self.linear_speed
                                     
        elif self.left_sensor < too_close and self.right_sensor > dist_to_wall:

            # msg.linear.x= 0
            msg.angular.z= self.angular_speed_slow
        
        elif self.left_sensor < dist_to_wall and self.right_sensor > dist_to_wall:

            msg.linear.x= self.linear_speed
            msg.angular.z= self.angular_speed_slow +0.15
              
        elif self.left_sensor < dist_to_wall and self.right_sensor < dist_to_wall:
            # msg.linear.x= 0
            msg.angular.z= self.angular_speed_slow

        else:
            pass

        self.cmd_pub.publish(msg)

    def bug2(self):
     
        if self.start_goal_line_calculated==False:
         
            self.robot_mode= "find goal"            
 
            self.start_x= self.current_x
            self.goal_x= self.goal_x
            self.start_y= self.current_y
            self.goal_y= self.goal_y
             

            self.goal_line_slope= ((self.goal_y-self.start_y) / (self.goal_x-self.start_x))
             

            self.goal_line_intercept= (
                self.goal_y-(self.goal_line_slope * self.goal_x))
                 

            self.start_goal_line_calculated= True
             
        if self.robot_mode=="find goal":
            self.go_to_goal()     

        elif self.robot_mode=="Follow the wall":
            self.follow_wall()

        elif self.robot_mode=="done":
            pass


def main(args=None):

    rclpy.init(args=args)
    bug_node= Bug2()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
