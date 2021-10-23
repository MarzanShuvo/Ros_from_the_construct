#!/usr/bin/env python
# license removed for brevity
import time
import rospy
from std_msgs.msg import String
from move_robot import MoveKobuki
from geometry_msgs.msg import Twist
from follow_line_step_multiple_complete import LineFollower

class LineObjective(object):

    def __init__(self):
        # There are two modes, line and star
        self.search_mode = "line"
        self.lost_star = 0
        self.star_found = False
        self.objective_tp = "/objective"
        self._check_objective_ready()
        rospy.Subscriber(self.objective_tp, String, self.objective_clb)

        # Moverof Turtlebot
        self.movekobuki_object = MoveKobuki()


        # Line Followers
        self.init_line_followers()
        self.set_line_folowers_states()


        self.rate = rospy.Rate(5)
        self.ctrl_c = False
                
        rospy.on_shutdown(self.shutdownhook)

    
    def init_line_followers(self):

        line_yellow_right = LineFollower(star_color="yellow", go_right=True)
        line_yellow_left = LineFollower(star_color="yellow", go_right=False)
        line_red = LineFollower(star_color="red")
        line_green = LineFollower(star_color="green")
        line_blue = LineFollower(star_color="blue")

        self.line_folowers_dict = {"yellow_left": line_yellow_left,
                                    "yellow_right": line_yellow_right,
                                    "red": line_red,
                                    "green": line_green,
                                    "blue": line_blue}
        
        print("Init Done="+str(self.line_folowers_dict))

    def clean_line_folowers(self):

        for key, value in self.line_folowers_dict.items():
            value.clean_up()



    def set_line_folowers_states(self):
        self.search_mode = "line"
        for key, value in self.line_folowers_dict.items():
            # print("key="+str(key)+",mission_objective="+str(self.mission_objective))
            if key == self.mission_objective:
                print("Activating="+str(key))
                value.set_detection_state(True)
            else:
                # If the mission is not yellow
                if "yellow" not in self.mission_objective and key=="yellow_left":
                    print("Activating Yellow Left for Start Search="+str(key))
                    value.set_detection_state(True)
                    self.search_mode = "star_"+str(self.mission_objective)
                else:
                    # We deactivate the detection 
                    value.set_detection_state(False)
            
    
    def get_cmd_vel(self):
        cmd_vel = Twist()
        for key, value in self.line_folowers_dict.items():
            if key == self.mission_objective:
                cmd_vel = value.get_current_twist()
                if "star" in self.search_mode :
                    # We see if The Star_red LineFollow is detecting something
                    if cmd_vel.linear.x != 0.0:
                        rospy.logwarn("STAR FOUND USING ITS CMD_VEL VALUES="+str(cmd_vel.linear.x))                       
                        # Finish script
                        self.star_found = True
                    else:
                        # We then have to get the cmd_vel form the line_follow yellow_left
                        if self.lost_star >= 1:
                            rospy.logwarn("STAR FOUND AND LOST...FINISHING")
                            time.sleep(3)
                            self.ctrl_c = True
                            
                        
                        rospy.logwarn("STAR_"+str(self.search_mode)+", WAS LOST, num="+str(self.lost_star))
                        cmd_vel = self.line_folowers_dict["yellow_left"].get_current_twist()
                        # Only if we found the start once at least we start counting if we lose it
                        if self.star_found:
                            self.lost_star +=1
            else:
                pass
        
        return cmd_vel

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
            self.movekobuki_object.clean_class()
            rospy.loginfo("shutdown time!")
            self.ctrl_c = True
    
    def loop(self):

        while not self.ctrl_c:
            rospy.loginfo("Mission=="+str(self.mission_objective))
            self.set_line_folowers_states()
            cmd_vel = self.get_cmd_vel()
            # rospy.loginfo("cmd_vel=="+str(cmd_vel))
            if cmd_vel.linear.x == 0.0:
                # Recovery mode
                rospy.logwarn("LOST, RECOVERY MODE")
                cmd_vel.linear.x = 0.2
                cmd_vel.angular.z = 0.1
            else:
                pass
            self.movekobuki_object.move_robot(cmd_vel)
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