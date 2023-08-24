##############################################################################
# Documentation
##############################################################################
"""
Custom behaviors used in order to control the curio robot.
"""
##############################################################################
# Imports
##############################################################################

import py_trees
import random
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import math
import std_msgs.msg as std_msgs
import time

##############################################################################
# Behaviours
##############################################################################

class ChangePath(py_trees.behaviour.Behaviour):
    """
    Behavior causes the robot to change the path taken.
    Returns SUCCESS if we find  a good path, FAILURE if
    we cycle through all paths and none work.

    Args:
        name (:obj:`str`): name of the behaviour

    Attributes:
        blackboard (:obj:`Blackboard`): Blackboard for behavior tree communication
        n_path (:obj:`int`): number of paths
        curr_path (:obj:`int`): id of the current path
        client (:obj:`Publisher`) : ROS publisher that alerts movebase client
    
    """
    def __init__(self, name):
        super(ChangePath, self).__init__(name=name)
        # Initialize blackboard with its variables
        self.blackboard = py_trees.blackboard.Blackboard()
        # Command to use a certain path
        self.blackboard.command = "None"
        # Current path feedback
        self.blackboard.curr_path = 1

    def setup(self, timeout):
        # Set up publisher to let movebase client know when to switch paths
        self.publisher = rospy.Publisher("move_base/switch_path", std_msgs.String, queue_size=1, latch=False)
        # Path 0 is always home so start at 1
        self.curr_path =1
        # Currently we have 3 paths
        self.n_path = 3
        return True
    
    def initialise(self):
        # Whenever ticked we want it to check all paths
        # Therefore start with 1
        self.curr_path = 1

    def update(self):
        # Can work in two modes
        # First just switch to the path you're commanded to
        if self.blackboard.command != "None":
            self.curr_path = int(self.blackboard.command)
            self.blackboard.command = "None"
        # Second you can put it after a PathClear check and
        # try another path to see if the path is clear
        else:
            if self.n_path > self.curr_path + 1:
                self.curr_path += 1
            else:
                rospy.loginfo("No paths left to try")
                return py_trees.common.Status.FAILURE
        # Publish the path and update the blackboard
        self.publisher.publish(std_msgs.String(str(self.curr_path)))
        self.blackboard.curr_path = self.curr_path
        return py_trees.common.Status.SUCCESS
        
class GoHome(py_trees.behaviour.Behaviour):
    """
    Behavior causes the robot to change the path to go home.

    Args:
        name (:obj:`str`): name of the behaviour

    Attributes:
        publisher (:obj:`Publisher`) : ROS publisher that alerts movebase node to switch paths
        blackboard (:obj:`Blackboard`): Blackboard for behavior tree communication
        curr_path (:obj:`int`): id of the current path
    
    """
    def __init__(self, name):
        super(GoHome, self).__init__(name=name)

    def setup(self, timeout):
        # Set up the publisher that will send command to switch path
        self.publisher = rospy.Publisher("move_base/switch_path", std_msgs.String, queue_size=10, latch=True)
        # Set up blackboard to communicate with other nodes
        self.blackboard = py_trees.blackboard.Blackboard()
        # Home path is always the first
        self.curr_path = 0
        return True

    def update(self):
        # Current path is now going home
        self.blackboard.curr_path = self.curr_path
        self.publisher.publish(std_msgs.String(str(self.curr_path)))
        return py_trees.common.Status.SUCCESS

class FollowPath(py_trees.behaviour.Behaviour):
    """
    Behavior keeps the robot following the path.

    Args:
        name (:obj:`str`): name of the behaviour

    Attributes:
        blackboard (:obj:`Blackboard`): Blackboard for behavior tree communication
    

    """
    def __init__(self, name):
        super(FollowPath, self).__init__(name=name)

    def setup(self, timeout):
        # Set up blackboard to communicate with other nodes
        self.blackboard = py_trees.blackboard.Blackboard()
        # Start with a pending state
        self.blackboard.state = GoalStatus.PENDING
        return True

    def update(self):
        # Based on the current state I have succeeded, failed or am currently
        # following the path
        state = int(self.blackboard.state)
        #print(state)
        #rospy.sleep(0)
        if state == GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        elif state == GoalStatus.ACTIVE or state == GoalStatus.PENDING or state == 2:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        # If I stopped when my goal was active
        # there was an interruption
        state = int(self.blackboard.state)
        if state == GoalStatus.ACTIVE:
            rospy.loginfo("Goal was preempted")

class Pause(py_trees.behaviour.Behaviour):
    """
    Behavior causes the robot to pause. It will resume the previous
    path when unpaused.

    Args:
        name (:obj:`str`): name of the behaviour

    Attributes:
        publisher (:obj:`Publisher`) : ROS publisher that alerts movebase node to pause
        blackboard (:obj:`Blackboard`): Blackboard for behavior tree communication
        curr_path (:obj:`int`): id of the current path
        paused (:obj:`boolean`): if we have paused or not
    

    """
    def __init__(self, name):
        super(Pause, self).__init__(name=name)

    def setup(self, timeout):
        # Set up publisher
        self.publisher = rospy.Publisher("move_base/switch_path", std_msgs.String, queue_size=10, latch=True)
        # This will let the movebase client know to pause
        self.curr_path = -1
        # To save the current path when we pause we use the blackboard
        self.blackboard = py_trees.blackboard.Blackboard()
        # To know if we have paused or not we use a blackboard var
        self.blackboard.pause = False
        self.paused = False
        return True
    
    def initialise(self):
        # At the start we have not sent the goal
        self.goal_sent = False

    def update(self):
        # When ticked if not paused it fails
        if not self.blackboard.pause and not self.goal_sent:
            return py_trees.common.Status.FAILURE
        # If the pause signal has not been send yet then send it
        # After sending toggle the booleans
        if not self.goal_sent:
            # If robot was not paused then we send pause signal
            # if not then we resume by sending the saved path
            if not self.paused:
                self.publisher.publish(std_msgs.String(str(self.curr_path)))
                self.blackboard.pause = False
                self.paused = True
            else:
                self.publisher.publish(std_msgs.String(str(self.blackboard.curr_path)))
                self.paused = False
            self.goal_sent = True
        else:
            # If you already sent the goal then run until
            # you get the signal again
            if self.blackboard.pause == False:
                return py_trees.common.Status.RUNNING
            else:
                # Resume path by sending saved value and resetting booleans
                self.publisher.publish(std_msgs.String(str(self.blackboard.curr_path)))
                self.goal_sent = False
                self.paused = False
                self.blackboard.pause = False
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
            
class Ask4Help(py_trees.behaviour.Behaviour):
    """
    Behavior causes the robot to flash a button on its dashboard asking for help.
    When the path is clear again it should resume its normal path.

    Args:
        name (:obj:`str`): name of the behaviour

    Attributes:
        publisher (:obj:`Publisher`) : ROS publisher that alerts movebase node to pause
        blackboard (:obj:`Blackboard`): Blackboard for behavior tree communication
        curr_path (:obj:`int`): id of the current path
        goal_sent (:obj:`boolean`): if we have sent the signal or not
    

    """
    def __init__(self, name):
        super(Ask4Help, self).__init__(name=name)

    def setup(self, timeout):
        # Set up publisher
        self.publisher = rospy.Publisher("/helpme", std_msgs.String, queue_size=10, latch=True)
        self.blackboard = py_trees.blackboard.Blackboard()
        # To know if the robot is stuck or not we use a blackboard var
        self.blackboard.pathclear = True
        self.publisher_p = rospy.Publisher("move_base/switch_path", std_msgs.String, queue_size=10, latch=True)
        # This will let the movebase client know to pause
        self.curr_path = -1
        self.paused = False
        return True
    
    def initialise(self):
        # At the start we have not sent the goal
        self.goal_sent = False

    def update(self):
        # If the signal has not been sent yet then send it
        # After sending toggle the booleans
        if int(self.blackboard.state) == 3:
            return py_trees.common.Status.FAILURE 
        # Variable is opposite
        # If clear then thank the person and return Failure
        if not self.blackboard.pathclear:
            self.publisher.publish(std_msgs.String("Thanks!"))
            self.publisher_p.publish(std_msgs.String(str(self.blackboard.curr_path)))
            return py_trees.common.Status.FAILURE
        # If not clear then ask for help and stay running
        if not self.goal_sent:
                self.publisher.publish(std_msgs.String("HELPME"))
                self.publisher_p.publish(std_msgs.String(str(self.curr_path)))
                self.goal_sent = True
            
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.publisher.publish(std_msgs.String("Thanks!"))
            
class TriggerRB(py_trees.behaviour.Behaviour):
    """
    Behavior causes the robot to go into recovery mode. It will resume the previous
    path when unpaused.

    Args:
        name (:obj:`str`): name of the behaviour

    Attributes:
        publisher (:obj:`Publisher`) : ROS publisher that alerts movebase node to pause
        blackboard (:obj:`Blackboard`): Blackboard for behavior tree communication
        curr_path (:obj:`int`): id of the current path
        paused (:obj:`boolean`): if we have paused or not
    

    """
    def __init__(self, name):
        super(TriggerRB, self).__init__(name=name)

    def setup(self, timeout):
        # Set up publisher
        self.publisher = rospy.Publisher("move_base/switch_path", std_msgs.String, queue_size=10, latch=True)
        # This will let the movebase client know to pause
        self.curr_path = -2
        # To save the current path when we pause we use the blackboard
        self.blackboard = py_trees.blackboard.Blackboard()
        self.start_time = 0.0
        return True
    
    def initialise(self):
        # At the start we have not sent the goal
        self.goal_sent = True
        self.resumed = False
        self.start_time = time.time()

    def update(self):
        # When ticked if not stuck it succeeds and resumes the path
        if int(self.blackboard.state) == GoalStatus.SUCCEEDED: 
            return py_trees.common.Status.FAILURE
        # If the trigger signal has not been send yet then send it
        # After sending toggle the booleans
        if not self.goal_sent:
            self.publisher.publish(std_msgs.String(str(self.curr_path)))
            self.start_time = time.time()
            self.goal_sent = True
        count = time.time() - self.start_time
        if count > 2 and not self.resumed:
            self.publisher.publish(std_msgs.String(str(self.blackboard.curr_path)))
            self.resumed = True
        if count > 7: 
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
            