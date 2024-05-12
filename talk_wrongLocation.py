#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal
class GenerationFunction():
    def __init__(self):
        self.client = SimpleActionClient('/tts', TtsAction)
        self.client.wait_for_server()
        rospy.loginfo("Tts connected!")
        self.goal = TtsGoal()
    def say_hi(self):
        try:
            self.goal.rawtext.text = "You have put the piece in a wrong location."
            self.goal.rawtext.lang_id = "en_GB"
            self.client.send_goal_and_wait(self.goal)
            # rospy.loginfo("GPT response is: Hi")
            # return "Hi"
        except Exception as e:
            rospy.logerr(f"Failed to say: {e}")
            # return "Error trying to say Hi."
if __name__ == '__main__':
    rospy.init_node('say_wrong_robot')
    gen_func = GenerationFunction()
    gen_func.say_hi()
    # rospy.loginfo(f"Robot said: {response}")