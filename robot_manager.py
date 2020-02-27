#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import sys
from src.utils import *
from minmax_pomdp.srv import *
#from naoqi import ALProxy, ALBroker, ALModule
import pdb

NODE_NAME = 'minmax_robot_manager'
pt_action_dic = {'up': "up", 'down':"down", 'left':"left", 'right':"right", 'up_left':"up and left", 'up_right':"up and right",
                 'down_left':"down and left", 'down_right':"down and right"}

pomdp_dic = {'no-action':0, 'interactive-tutorial':1, 'worked-example':2, 'hint':3, 'think-aloud':4, 'break':5}
        #   - no-action
        #   - interactive-tutorial
        #   - worked-example
        #   - hint
        #   - think-aloud
        #   - break
"""
Module responsible for the Robot's Conections and Behaviors


"""


class RobotManager:

    def __init__(self):
        # Explanation Variables
        self.depth = None
        self.ball = None
        self.action = None
        self.score = None
        self.best_ball = None
        self.best_action = None
        self.best_score = None
        self.num_turns = None
        #self.pomdp_action = None
        self.explanation_text = None

        # Services and client (of game manager)
        self.robot_explanation_service = rospy.Service('robot_explanation_service', RobotExplanation,
                                                       self.handle_robot_explanation_service)

        rospy.wait_for_service('get_pomdp_action')
        try:
            self.get_pomdp_action_proxy = rospy.ServiceProxy('get_pomdp_action', PomdpAction)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        # #Robot Connection Variables and Services
        # #self.nao_IP = '192.168.1.101'
        # #self.nao_port = 9559

        # self.myBroker = ALBroker("myBroker",
        #                      "0.0.0.0",  # listen to anyone
        #                      0,  # find a free port and use it
        #                      self.nao_IP,  # parent broker IP
        #                      9559)  # parent broker port)

        # self.robot_communication = ALProxy("ALTextToSpeech", self.nao_IP, self.nao_port)
        # self.robot_communication.setLanguage('Portuguese')
        # self.robot_communication.setLanguage('English')
        # self.robot_communication.setVolume(0.7)
        # self.robot_communication.setParameter("speed", 70)

        # #self.faceProxy = ALProxy("ALFaceDetection", self.nao_IP, self.nao_port)
        # self.animatedSpeechProxy = ALProxy("ALAnimatedSpeech", self.nao_IP, self.nao_port)
        # self.tracker = ALProxy("ALTracker", self.nao_IP, self.nao_port)
        # self.motionProxy = ALProxy("ALMotion", self.nao_IP, self.nao_port)
        # self.ledProxy = ALProxy("ALLeds", self.nao_IP, self.nao_port)
        # self.postureProxy = ALProxy("ALRobotPosture", self.nao_IP, self.nao_port)

        # Setup Robot
        #self.wake_robot()



    def handle_robot_explanation_service(self, req):

        # Read the request
        self.depth = req.depth
        self.ball = req.ball
        self.action = req.action
        self.score = req.score
        self.best_ball = req.best_ball
        self.best_action = req.best_action
        self.best_score = req.best_score

        # Count number of turns
        self.num_turns = int((self.depth - 1)/2.0)

        # Generate Explanation
        self.explanation_text = self.generate_explanation()

        # Send explanation text to the robot
        #self.robot_communication.say(self.explanation_text)
        print(self.explanation_text)

        # Clean explanation variables
        self.clean_explanation_variables()

        # Respond to service
        rexplan_resp = RobotExplanationResponse()
        rexplan_resp.success = True
        return rexplan_resp



    def generate_explanation(self):
        pomdp_action = self.get_pomdp_action_proxy.call().pomdp_action

        explanation = "\\rspd=80\\"

        if pomdp_action == pomdp_dic["no-action"]:
            explanation += "You did great!"
        elif pomdp_action == pomdp_dic["interactive-tutorial"]:
            explanation += "I did the wrong move. Can you show me how could I make it better?"
        elif pomdp_action == pomdp_dic["worked-example"]:
            explanation += "Oh! I made a mistake. Let me show another solution."
        elif pomdp_action == pomdp_dic["hint"]:
            explanation += "Oh! I should have move differently. I obtained " + str(self.score) + " points \\pau=700\\ but I could have get " + str(self.best_score) + " points\\pau=700\\. Now it is your turn."
        elif pomdp_action == pomdp_dic["break"]:
            explanation += "Oh no! I am a bit tired. Would you like to take a break?"
        elif pomdp_action == pomdp_dic["think-aloud"]:
            explanation += "Oh! I did wrong. I moved the ball " + str(self.ball) + " to the " + str(
                    pt_action_dic[get_action_name(self.action)]) \
                    + " \\pau=50\\ and I obtained " + str(self.score) + " points \\pau=700\\ but moving the ball \\pau=50\\ " \
                    + str(self.best_ball) + " \\pau=50\\ to the" + str(pt_action_dic[get_action_name(self.best_action)]) \
                    + " \\pau=50\\ I could have get " + str(self.best_score) + " points\\pau=700\\. Now it is your turn."
        # actions:
        #   - no-action
        #   - interactive-tutorial
        #   - worked-example
        #   - hint
        #   - think-aloud
        #   - break
        # if self.num_turns == 0:
        #
        #     explanation += "Oh! I did wrong. I moved the ball " + str(self.ball) + " to the " + str(
        #         pt_action_dic[get_action_name(self.action)]) \
        #                    + " \\pau=50\\ and I obtained " + str(self.score) + " points \\pau=700\\ but moving the ball \\pau=50\\ " \
        #                    + str(self.best_ball) + " \\pau=50\\ to the" + str(pt_action_dic[get_action_name(self.best_action)]) \
        #                    + " \\pau=50\\ I could have get " + str(self.best_score) + " points\\pau=700\\. Now it is your turn."
        #
        #
        # elif self.num_turns == 1:
        #
        #     explanation += "I am not doing well. I moved the ball " + str(self.ball) + " to the " + str(pt_action_dic[get_action_name(self.action)])\
        #                    + " \\pau=50\\  and I will obtain " + str(self.score) + " points in the next turn\\pau=700\\ But\\pau=50\\ moving the ball "\
        #                    + str(self.best_ball) + " to the " + str(pt_action_dic[get_action_name(self.best_action)])\
        #                    + " \\pau=50\\ I could have get " + str(self.best_score) + " points in the next turn \\pau=700\\. Now it is your turn."
        # else:
        #
        #     explanation += "I did a mistake. I moved the ball " + str(self.ball) + " to the " + str(pt_action_dic[get_action_name(self.action)]) \
        #                    + " and I'm going to obtain  \\pau=50\\ " + str(self.score) + "\\pau=50\\  points in the next " + str(self.num_turns) + " turns \\pau=700\\ but\\pau=50\\ moving the ball " \
        #                    + str(self.best_ball) + " \\pau=50\\ to the" + str(pt_action_dic[get_action_name(self.best_action)]) \
        #                    + " \\pau=50\\ I could have get " + str(self.best_score) + " \\pau=50\\ points in the next " + str(self.num_turns) + " turns \\pau=700\\. Now it is your turn."



        return explanation





    def clean_explanation_variables(self):
        self.depth = None
        self.ball = None
        self.action = None
        self.score = None
        self.best_ball = None
        self.best_action = None
        self.best_score = None
        self.num_turns = None
        self.explanation_text = None
        #self.pomdp_action = None
        return


    def wake_robot(self):
        # Turn on Leds
        self.setup_leds(turn_on=True)

        # Turn on Face Tracking
        self.setup_face_tracking(0.1, turn_on=True)



    def setup_face_tracking(self, faceSize, turn_on=True):
        """ Robot starts to track the users face
        """


        if turn_on:

            # First, wake up
            self.motionProxy.wakeUp()
            self.motionProxy.rest()

            # Introduction
            self.postureProxy.goToPosture("Crouch", 1.0)
            self.motionProxy.setBreathEnabled('Arms', True)
            self.animatedSpeechProxy.say(
                 "^startTag(hello) Hi, I am NAO, ^startTag(You_1) and you?  What's your name? \
                                              \\pau=1500\\ \
                                              ^startTag(Yes_1) Nice to meet you.\
                                              \\pau=600\\ \
                                              Do you know the game Minicomputer Tug of War? \
                                              \\pau=1500\\ \
                                              No problem, we can learn it together ^startTag(Enthusiastic_4)\
                                              \\pau=600\\ \
                                              Ready? I'm going to tell you how to play\
                                              \\pau=600\\ \
                                              The game uses the Papi Minicomputer ^startTag(Explain_1)\
                                              \\pau=600\\ \
                                              it has two players: ^startTag(You_2) you\\pau=80\\ and ^startTag(Me_1) \\pau=80\\me, that alternates \
                                              \\pau=600\\ \
                                              As you can see from the screen, one player starts with 1000 points, the other one starts with 5. ^startTag(Explain_8)\
                                              \\pau=600\\ \
                                              It is possible to move only one ball at the time\\pau=80\\ and\\pau=80\\and have only one ball of the same color in each square\\pau=180\\. You can move only to the closest square \
                                              \\pau=1000\\ \
                                              The player with the white balls has to get less or equal points\\pau=50\\ of the adversary. ^startTag(Explain_6)\
                                              \
                                              \\pau=600\\ \
                                              The player with the black balls has to get more or equal points of the adversary.  \
                                              \
                                              \\pau=600\\ \
                                              At least, those are the rules that I understood. Did you get them?\
                                              \\pau=1000\\ \
                                              Great! Let's start")


            self.postureProxy.goToPosture("Crouch", 1.0)

            #self.animatedSpeechProxy.say("hey! ^startTag(hello)")


            # Add target to track
            targetName = "Face"
            faceWidth = faceSize
            self.tracker.registerTarget(targetName, faceWidth)

            # Then, start tracker
            self.tracker.track(targetName)

        else:
            self.tracker.stopTracker()
            self.tracker.unregisterAllTargets()


    def setup_leds(self, turn_on=True):

        if turn_on:
            section1 = ["FaceLeds", "ChestLeds"]
            self.ledProxy.createGroup("turn", section1)
            self.ledProxy.fadeRGB("turn", 0x00FFFFFF, 0.3)
        else:
            section1 = ["FaceLeds", "ChestLeds"]
            self.ledProxy.createGroup("turn", section1)
            self.ledProxy.fadeRGB("turn", 0x00000000, 0.3)



if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    rospy.loginfo("[Robot Manager] Node is running...")
    rm = RobotManager()
    rospy.spin()
