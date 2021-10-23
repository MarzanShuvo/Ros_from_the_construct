#!/usr/bin/env python
# license removed for brevity
import time
import rospy
from std_msgs.msg import String
from move_robot import MovePhantomX
from geometry_msgs.msg import Twist
from follow_line_step_multiple_complete import LineFollower

class LineObjective(object):

    def __init__(self):
        # There are two modes, line and star

        self.star_found_counter = 0
        self.star_found = False
        self.objective_tp = "/objective"
        self._check_objective_ready()
        rospy.Subscriber(self.objective_tp, String, self.objective_clb)

        # Moverof Turtlebot
        self.MovePhantomX_object = MovePhantomX()


        # Line Followers
        self.init_line_followers()

        self.rate = rospy.Rate(5)
        self.ctrl_c = False
                
        rospy.on_shutdown(self.shutdownhook)

    
    def init_line_followers(self):

        line_green_right = LineFollower(star_color="green", go_right=True, show_crop=True)
        line_green_left = LineFollower(star_color="green", go_right=False, show_crop=False)
        line_red = LineFollower(star_color="red", go_right=True, show_crop=False)
        line_yellow = LineFollower(star_color="yellow", go_right=True, show_crop=False)
        line_blue = LineFollower(star_color="blue", go_right=True, show_crop=False)

        self.line_folowers_dict = {"green_left": line_green_left,
                                    "green_right": line_green_right,
                                    "red": line_red,
                                    "yellow": line_yellow,
                                    "blue": line_blue}
        
        print("Init Done="+str(self.line_folowers_dict))

        self.activate_all_detectors()
    
    def activate_all_detectors(self):

        for key, value in self.line_folowers_dict.items():
            print("Starting_Activating="+str(key))
            value.set_detection_state(True)


    def clean_line_folowers(self):

        for key, value in self.line_folowers_dict.items():
            value.clean_up()


    def objective_clb(self,msg):
        self.mission_objective = msg.data

    def _check_objective_ready(self):
        self.mission_objective_msg = None
        while self.mission_objective_msg is None and not rospy.is_shutdown():
            try:
                self.mission_objective_msg = rospy.wait_for_message(self.objective_tp, String, timeout=1.0)
                rospy.logdebug("Current "+self.objective_tp+" READY=>" + str(self.mission_objective_msg))

            except:
                rospy.logerr("Current "+self.objective_tp+" not ready yet, retrying.")
        
        self.mission_objective = self.mission_objective_msg.data
        

    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        rospy.loginfo("Cleaning and STopping Turtlebot")
        self.clean_line_folowers()
        self.MovePhantomX_object.clean_class()
        rospy.loginfo("shutdown time!")
        self.ctrl_c = True
    

    def check_star_colour(self,star_color):
        cmd_vel, found_color = self.line_folowers_dict[star_color].get_current_twist()
        return found_color, cmd_vel

    def loop(self):

        while not self.ctrl_c:
            rospy.loginfo("Mission=="+str(self.mission_objective))
            if "green" not in self.mission_objective: 
                found_star, cmd_vel_star = self.check_star_colour(self.mission_objective)
            else:
                found_star = False
            
            found_line, cmd_vel_line = self.check_star_colour("green_right")

            if found_star:
                rospy.logwarn("STAR FOUND..."+str(self.star_found_counter))
                self.star_found_counter += 1               
                self.MovePhantomX_object.move_robot(cmd_vel_star)
            else:
                if self.star_found_counter > 10:
                    # You found the star and lost it so you have to be close, stop then
                    rospy.logwarn("STAR FOUND AND LOST...FINISHING")
                    time.sleep(3)
                    self.ctrl_c = True
                else:
                    if found_line:
                        # We follow the line if no star found
                        rospy.logwarn("LINE FOUND...")
                        self.MovePhantomX_object.move_robot(cmd_vel_line)
                    else:
                        # We start the recovery mode spiral movement
                        rospy.logwarn("LOST, RECOVERY MODE")
                        rec_cm_vel = Twist()
                        rec_cm_vel.linear.x = 0.3
                        rec_cm_vel.angular.z = 0.1
                        self.MovePhantomX_object.move_robot(rec_cm_vel)
            
            self.rate.sleep()


def main():
    
    rospy.init_node('line_objective', anonymous=True, log_level=rospy.DEBUG)
    line_obj = LineObjective()
    line_obj.loop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass