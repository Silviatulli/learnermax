# Minmax POMDP

1) Install ROS melodic and catkin following this link: https://wiki.ros.org/catkin#Installing_catkin
2) Clone the repository 
    - $ git clone https://github.com/Silviatulli/minmax.git 
    - add an __init__.py file into the folders srv and src for simplifying the research of the folders' content 
2) Remember to make the python files executable by running the following command for each script:
    - $ chmod +x filename.py
3) Build a catkin workspace and source the setup file run:
    - $ cd ~/catkin_ws
    - $ catkin_make
4) add the workspace to the ROS environment:
    - $. ~/catkin_ws/devel/setup.bash
5) Make sure that the CMakeLists.txt file is configured properly, with all the services and the dependencies listed as follows:
    - find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      message_generation
      message_runtime
    )
    - add_service_files(
       FILES
       Decision.srv
       GameState.srv
       Plan.srv
       RobotExplanation.srv
       RobotTalk.srv
     )

6) If you work with the NAO Robot uncomment the line 7, from 41 to 65 and 85 (self.robot_communication.say(self.explanation_text)) in the file robot_manager.py.
Create a folder sdk that contains the pynaoqi sdk required and modify your bashrc ($gedit ~/.bashrc) to indicate the python and library path as following:
- export PYTHONPATH=$PYTHONPATH:~/sdk/pynaoqi-python2.7-2.1.2.17-linux64
- export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/<your_pc_name>/sdk/pynaoqi-python2.7-2.1.2.17-linux64

7) you can download the pynaoqi sdk following this guide: http://wiki.ros.org/nao/Tutorials/Installation

8) Launch the code: 
- open a second terminal launch minmaxpomdp.launch: $ roslaunch minmax.launch

# POMDP dependencies
1) Install numpy.
2) The python code in this repository refers to the work implemented by Ramachandran Sebo and Scasellati (2019) and will look for the pomdp-solve executable in your $PATH. The POMDP solver used to create AT-POMDP was originally implemented by Kaelbling, Littman, and Cassandra (1998) and was modified by Roncone, Mangin, and Scassellati (2017). Please refer to their instructions on installing this executable by reading the section titled "Prerequisites for using the POMDP solvers" and utilizing the code they reference at (https://github.com/scazlab/pomdp-solve). If you are using their modified solver, we recommend that you cite their paper:

- Aditi Ramachandran, Sarah Strohkorb Sebo, Brian Scassellati (2019). Personalized Robot Tutoring using the Assistive Tutor POMDP (AT-POMDP). In Proceedings of The Thirty-Third AAAI Conference on Artificial Intelligence (AAAI 2019). Honolulu, HI, USA.

- Roncone Alessandro, Mangin Olivier, Scassellati Brian (2017). Transparent Role Assignment and Task Allocation in Human Robot Collaboration. IEEE International Conference on Robotics and Automation (ICRA 2017), Singapore.


if you have any doubts please do not hesitate to contact me by e-mail
