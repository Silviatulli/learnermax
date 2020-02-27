#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
import sys
from src.utils import *
from minmax_pomdp.srv import *
sys.path.append("/home/isir/src/task-models")
import json
import numpy as np
from task_models.lib.pomdp import POMDP, GraphPolicyBeliefRunner
from pomdp_setup_reward_transition_matrices import *
from pomdp_setup_observation_matrices import *

import pdb

NODE_NAME = 'minmax_game_module'

"""
Module responsible for the Game
"""

pomdp_dic = {'no-action':0, 'interactive-tutorial':1, 'worked-example':2, 'hint':3, 'think-aloud':4, 'break':5}


def test_command_line_sequence(self, param_file):
    #read in params
    #pdb.set_trace()

    # discount factor
    discount = rospy.get_param('discount')
    print(discount)

    # state variables
    knowledge_states = rospy.get_param('knowledge_states')
    engagement_states = rospy.get_param('engagement_states')
    attempt_states = rospy.get_param('attempt_states')
    num_knowledge_levels = len(knowledge_states)
    num_engagement_levels = len(engagement_states)
    num_attempts = len(attempt_states)
    all_states = combine_states_to_one_list(knowledge_states, engagement_states, attempt_states)
    num_states = len(all_states)

    # starting distribution
    start = np.zeros(num_states)
    num_start_states = num_knowledge_levels * num_engagement_levels
    for i in range(num_states):
        if i%num_attempts == 0:
            start[i] = 1.0 / float(num_start_states)
        else:
            start[i] = 0.0

    # probabilities associated with the transition matrix
    prob_knowledge_gain = rospy.get_param('prob_knowledge_gain')
    prob_engagement_gain = rospy.get_param('prob_engagement_gain')
    prob_engagement_loss = rospy.get_param('prob_engagement_loss')
    prob_correct_answer = rospy.get_param('prob_correct_answer')
    prob_correct_answer_after_1_attempt = rospy.get_param('prob_correct_answer_after_1_attempt')
    prob_drop_for_low_engagement = rospy.get_param('prob_drop_for_low_engagement')

    # actions
    actions = rospy.get_param('actions')
    num_actions = len(actions)

    # action-related reward variables
    action_rewards = rospy.get_param('action_rewards')
    engagement_reward = rospy.get_param('engagement_reward')
    knowledge_reward = rospy.get_param('knowledge_reward')
    end_state_remain_reward = rospy.get_param('end_state_remain_reward')
    reward_for_first_attempt_actions = rospy.get_param('reward_for_first_attempt_actions')
    action_prob_knowledge_gain_mult = rospy.get_param('action_prob_knowledge_gain_mult')
    action_prob_engagement_gain_mult = rospy.get_param('action_prob_engagement_gain_mult')

    # observations
    correctness_obs = rospy.get_param('correctness_obs')
    speed_obs = rospy.get_param('speed_obs')
    all_obs = combine_obs_types_to_one_list(correctness_obs, speed_obs)
    num_observations = len(all_obs)

    # observation related variables
    prob_speeds_for_low_engagement = rospy.get_param('prob_speeds_for_low_engagement')
    prob_speeds_for_high_engagement = rospy.get_param('prob_speeds_for_high_engagement')
    action_speed_multipliers = np.array(rospy.get_param('action_speed_multipliers'))


    R = generate_reward_matrix(actions=actions,
                               action_rewards=action_rewards,
                               engagement_reward=engagement_reward,
                               knowledge_reward=knowledge_reward,
                               end_state_remain_reward=end_state_remain_reward,
                               num_knowledge_levels=num_knowledge_levels,
                               num_engagement_levels=num_engagement_levels,
                               num_attempts=num_attempts,
                               num_observations=num_observations,
                               reward_for_first_attempt_actions=reward_for_first_attempt_actions)

    T = generate_transition_matrix(num_knowledge_levels=num_knowledge_levels,
                                   num_engagement_levels=num_engagement_levels,
                                   num_attempts=num_attempts,
                                   prob_knowledge_gain=prob_knowledge_gain,
                                   prob_engagement_gain=prob_engagement_gain,
                                   prob_engagement_loss=prob_engagement_loss,
                                   action_prob_knowledge_gain_mult=action_prob_knowledge_gain_mult,
                                   action_prob_engagement_gain_mult=action_prob_engagement_gain_mult,
                                   prob_correct_answer=prob_correct_answer,
                                   prob_correct_answer_after_1_attempt=prob_correct_answer_after_1_attempt,
                                   prob_drop_for_low_engagement=prob_drop_for_low_engagement)

    O = generate_observation_matrix(knowledge_states=knowledge_states,
                                    engagement_states=engagement_states,
                                    attempt_states=attempt_states,
                                    correctness_obs=correctness_obs,
                                    speed_obs=speed_obs,
                                    num_actions=num_actions,
                                    prob_speeds_for_low_engagement=prob_speeds_for_low_engagement,
                                    prob_speeds_for_high_engagement=prob_speeds_for_high_engagement,
                                    action_speed_multipliers=action_speed_multipliers)


    #create POMDP model
    simple_pomdp = POMDP(T, O, R, np.array(start), discount, states=all_states, actions=actions,
                         observations=all_obs, values='reward')

    simple_pomdp_graph_policy = simple_pomdp.solve(method='grid', verbose=False, n_iterations=500)

    simple_pomdp_graph_policy_belief_runner = GraphPolicyBeliefRunner(simple_pomdp_graph_policy,
                                                                      simple_pomdp)


    num_states_per_knowledge_level = num_engagement_levels * num_attempts
    problem_num = 1
    attempt_num = 1
    #receiving_obs = True
    observations = ['R-fast', 'R-med', 'R-slow', 'W-fast', 'W-med', 'W-slow']
    for obs in observations:
        #while receiving_obs is True:
        obs = random.choice(observations)
        #raw_input("Enter observation: ")
        if obs == "done":
            receiving_obs = False
            break
        if obs not in all_obs:
            print "Invalid observation provided\n"
            continue
        knowledge_level_index = 0
        action = simple_pomdp_graph_policy_belief_runner.get_action()
        current_belief = simple_pomdp_graph_policy_belief_runner.step(obs)
        print "\nProblem %i, Attempt %i: (%s, %s)" % (problem_num, attempt_num, action, obs)


        belief_str = ""
        sum_across_states = 0.0
        for k in range(num_states):
            sum_across_states += current_belief[k]
            if k % num_attempts == num_attempts - 1:
                belief_str += "%s: %.3f\t\t" % (all_states[k][:-3], sum_across_states)
                knowledge_level_index += 1
                sum_across_states = 0.0
            if k % num_states_per_knowledge_level == num_states_per_knowledge_level-1:
                belief_str += "\n"

        print belief_str
        self.pomdp_action = action
        # list out keys and values separately
        self.key_list = list(pomdp_dic.keys())
        self.val_list = list(pomdp_dic.values())
        print(self.val_list[self.key_list.index(self.pomdp_action)])

        if "R" in obs or attempt_num == 3:
            problem_num += 1
            attempt_num = 1
        else:
            attempt_num += 1


        #print(pomdp_action)

class GameManager:

    def __init__(self):

        # Game manager Variable
        self.finished_game = None
        self.wait = None
        self.robot_turn = None
        self.child_turn = None
        self.robot_id = None
        self.child_id = None
        self.minimizer_ball_0 = None
        self.minimizer_ball_1 = None
        self.maximizer_ball_0 = None
        self.maximizer_ball_1 = None
        self.optimal_robot_decision = None
        self.process = None



        # Services
        self.game_to_manager_service = rospy.Service('game_to_manager_service', GameState, self.handle_game_to_manager)
        self.get_pomdp_action = rospy.Service('get_pomdp_action', PomdpAction, self.handle_pomdp_action)

        rospy.wait_for_service('decision_service')
        rospy.wait_for_service('robot_talk_service')
        rospy.wait_for_service('manager_to_game_service')
        try:
            self.decision_proxy = rospy.ServiceProxy('decision_service', Decision)
            self.robot_talk_proxy = rospy.ServiceProxy('robot_talk_service', RobotTalk)
            self.manager_to_game_proxy = rospy.ServiceProxy('manager_to_game_service', GameState)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


        # Setup Game
        self.start_game()


    def start_game(self):

        # Game manager Variable
        self.finished_game = False
        self.wait = False
        self.robot_turn = False
        self.child_turn = True
        self.robot_id = player_type_dic['min']
        self.child_id = player_type_dic['max']
        self.minimizer_ball_0 = 200
        self.minimizer_ball_1 = 800
        self.maximizer_ball_0 = 1
        self.maximizer_ball_1 = 4
        self.optimal_robot_decision = True
        self.process = False

        # Send to Game the information
        game_resp = self.send_state_to_game()
        return game_resp

    def ask_robot_to_talk(self, speech_type=0):

        robot_talk_request = RobotTalkRequest()
        robot_talk_request.speech_type = speech_type
        robot_talk_resp = self.robot_talk_proxy.call(robot_talk_request)
        return robot_talk_resp


    def ask_decision(self):

        decision_request = DecisionRequest()
        decision_request.optimal = self.optimal_robot_decision
        decision_request.maximizer_ball_0 = self.maximizer_ball_0
        decision_request.maximizer_ball_1 = self.maximizer_ball_1
        decision_request.minimizer_ball_0 = self.minimizer_ball_0
        decision_request.minimizer_ball_1 = self.minimizer_ball_1

        # If it is an optimal solution we do not ask for generation of explanation
        if self.optimal_robot_decision:
            decision_request.generate_explanation = False
        else:
            decision_request.generate_explanation = True  # THIS IS WHY IT WAS NOT GENERATING EXPLANATION WITH THE ROBOT!

        rospy.loginfo("[Game Manager] Sending information to Decision Manager")
        rospy.loginfo("[Game Manager] Optimal Decision = " + str(decision_request.optimal))
        rospy.loginfo("[Game Manager] Max Ball 0 = " + str(decision_request.maximizer_ball_0))
        rospy.loginfo("[Game Manager] Max Ball 1 = " + str(decision_request.maximizer_ball_1))
        rospy.loginfo("[Game Manager] Min Ball 0 = " + str(decision_request.minimizer_ball_0))
        rospy.loginfo("[Game Manager] Min Ball 1 = " + str(decision_request.minimizer_ball_1))

        # Send to the decision manager and get the response
        decision_resp = self.decision_proxy.call(decision_request)
        return decision_resp



    def send_state_to_game(self, finished_game=None, wait=None, robot_turn=None, child_turn=None, min_ball_0=None,
                           min_ball_1=None, max_ball_0=None, max_ball_1=None):

        # Send to Game the information
        if wait is None:
            game_request = GameStateRequest()
            game_request.finished_game = self.finished_game
            game_request.wait = self.wait
            game_request.robot_turn = self.robot_turn
            game_request.child_turn = self.child_turn
            game_request.maximizer_ball_0 = self.maximizer_ball_0
            game_request.maximizer_ball_1 = self.maximizer_ball_1
            game_request.minimizer_ball_0 = self.minimizer_ball_0
            game_request.minimizer_ball_1 = self.minimizer_ball_1

        else:
            game_request = GameStateRequest()
            game_request.finished_game = finished_game
            game_request.wait = wait
            game_request.robot_turn = robot_turn
            game_request.child_turn = child_turn
            game_request.maximizer_ball_0 = max_ball_0
            game_request.maximizer_ball_1 = max_ball_1
            game_request.minimizer_ball_0 = min_ball_0
            game_request.minimizer_ball_1 = min_ball_1

        rospy.loginfo("[Game Manager] Sending information to Game Interface")
        rospy.loginfo("[Game Manager] Finished Game = " + str(game_request.finished_game))
        rospy.loginfo("[Game Manager] Wait = " + str(game_request.wait))
        rospy.loginfo("[Game Manager] Robot Turn = " + str(game_request.robot_turn))
        rospy.loginfo("[Game Manager] Child Turn = " + str(game_request.child_turn))
        rospy.loginfo("[Game Manager] Max Ball 0 = " + str(game_request.maximizer_ball_0))
        rospy.loginfo("[Game Manager] Max Ball 1 = " + str(game_request.maximizer_ball_1))
        rospy.loginfo("[Game Manager] Min Ball 0 = " + str(game_request.minimizer_ball_0))
        rospy.loginfo("[Game Manager] Min Ball 1 = " + str(game_request.minimizer_ball_1))

        # Call the manager to game service and wait for response
        game_response = self.manager_to_game_proxy.call(game_request)
        return game_response.success


    def handle_game_to_manager(self, req):

        rospy.loginfo("[Game Manager] Receiving information from Game Interface")

        # Read Request and update variables
        self.finished_game = req.finished_game
        self.wait = req.wait
        self.robot_turn = req.robot_turn
        self.child_turn = req.child_turn
        self.maximizer_ball_0 = req.maximizer_ball_0
        self.maximizer_ball_1 = req.maximizer_ball_1
        self.minimizer_ball_0 = req.minimizer_ball_0
        self.minimizer_ball_1 = req.minimizer_ball_1

        rospy.loginfo("[Game Manager] Finished Game = " + str(self.finished_game))
        rospy.loginfo("[Game Manager] Wait = " + str(self.wait))
        rospy.loginfo("[Game Manager] Robot Turn = " + str(self.robot_turn))
        rospy.loginfo("[Game Manager] Child Turn = " + str(self.child_turn))
        rospy.loginfo("[Game Manager] Max Ball 0 = " + str(self.maximizer_ball_0))
        rospy.loginfo("[Game Manager] Max Ball 1 = " + str(self.maximizer_ball_1))
        rospy.loginfo("[Game Manager] Min Ball 0 = " + str(self.minimizer_ball_0))
        rospy.loginfo("[Game Manager] Min Ball 1 = " + str(self.minimizer_ball_1))


        # Tell the game manager we need to process information
        self.process = True
        rospy.loginfo("[Game Manager] Processing Information = " + str(self.process))

        # Send Response that we updated the variables
        gstate_resp = GameStateResponse()
        gstate_resp.success = True
        return gstate_resp

    def process_information(self):

        if not self.finished_game:

            if not self.process:
                return
            else:

                rospy.loginfo("[Game Manager] Processing information")
                # If it was the robot turn that ended, then we change to the child's turn
                if self.robot_turn:
                    self.robot_to_child_turn()

                # if it was the child turn that ended, we need to change to the robot's turn
                if self.child_turn:
                    self.child_to_robot_turn()

                self.process = False
        else:
            rospy.loginfo("[Game Manager] Finished Game")

    def robot_to_child_turn(self):

        rospy.loginfo("[Game Manager] Changing robot -> child turn")

        # Change robot to child turn
        self.child_turn = True
        self.robot_turn = False

        # Send message to game
        game_resp = self.send_state_to_game()
        if not game_resp:
            rospy.loginfo("[Game Manager] ERROR! Could not change robot -> child turn.")
        return


    def child_to_robot_turn(self):

        rospy.loginfo("[Game Manager] Changing child -> robot turn")
        rospy.loginfo("[Game Manager] Asking Game to freeze all balls.")

        # First thing, ask the game to freeze all balls
        self.wait = True
        game_resp = self.send_state_to_game()

        if not game_resp:
            rospy.loginfo("[Game Manager] ERROR! Could not freeze all balls.")

        # Next, ask the decision module, where should we move the balls! So we make a request
        decision_resp = self.ask_decision()

        # Read the response
        ball_to_move = decision_resp.ball
        new_ball_score = decision_resp.new_ball_score

        # if the ball to move is the ball 0, we update its score
        if ball_to_move == 0:
            self.minimizer_ball_0 = new_ball_score
        else:
            self.minimizer_ball_1 = new_ball_score

        # Send to game the new information
        self.wait = False
        self.robot_turn = True
        self.child_turn = False
        game_resp = self.send_state_to_game()

        if not game_resp:
            rospy.loginfo("[Game Manager]  ERROR! Could not Move balls.")

        # If we don't need to generate explanation it is finished
        if self.optimal_robot_decision:
            rospy.loginfo("[Game Manager] Optimal Decision was decided. Next turn it will be sub-optimal")
            self.optimal_robot_decision = False  # NEXT TURN IT WONT BE OPTIMAL

        # If not, we need to generate explanation
        else:
            rospy.loginfo("[Game Manager] Sub Optimal Decision was decided. Next turn it will be optimal.")
            self.optimal_robot_decision = True  # NEXT TURN IT WILL BE OPTIMAL

            # First thing, ask the game to freeze all balls
            rospy.loginfo("[Game Manager] Asking game interface to freeze balls.")
            self.wait = True
            game_resp = self.send_state_to_game()

            if not game_resp:
                rospy.loginfo("[Game Manager] ERROR! Could not Freeze all balls.")

            # Next ask the robot to talk
            rospy.loginfo("[Game Manager] Asking robot to generate explanation.")

            #Generate POMDP actions
            if len(sys.argv) > 1:
                 pm = test_command_line_sequence(self, sys.argv[1])
            else:
                print "please provide the name of the input parameter file as a command line argument"

            robot_resp = self.robot_talk_proxy.call(robot_speech_type_dic['explanation'])
            if not robot_resp:
                rospy.loginfo("[Game Manager] ERROR! Could not make robot explain.")

        # Next change to child's turn:
        self.wait = False
        self.robot_to_child_turn()
        return

    def handle_pomdp_action(self, req):
        return self.val_list[self.key_list.index(self.pomdp_action)]



if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    gm = GameManager()
    rospy.loginfo("[Game Manager] Node is running...")
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        gm.process_information()
        rate.sleep()
