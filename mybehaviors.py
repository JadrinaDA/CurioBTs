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
        self.publisher = rospy.Publisher("move_base/switch_path", std_msgs.String, queue_size=10, latch=True)
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
        if self.blackboard.command != "None":
            self.curr_path = int(self.blackboard.command)
            # if len(str(self.blackboard.command)) == 1 :
            #     self.curr_path = int(self.blackboard.command)
            # else:
            #     self.publisher.publish(self.blackboard.command)
            #     self.blackboard.command = "None"
            #     return py_trees.common.Status.SUCCESS
            self.blackboard.command = "None"
        else:
            if self.n_path > self.curr_path + 1:
                self.curr_path += 1
            else:
                rospy.loginfo("No paths left to try")
                return py_trees.common.Status.FAILURE
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
        if state == GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        elif state == GoalStatus.ACTIVE or state == GoalStatus.PENDING:
            return py_trees.common.Status.RUNNING
        else:
            print(state)
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
        # At the start we have not send the goal
        self.goal_sent = False

    def update(self):
        # When ticked if not paused it fails
        if not self.blackboard.pause:
            return py_trees.common.Status.FAILURE
        # If the pause signal has not been send yet then send it
        # After sending toggle the booleans
        if not self.goal_sent:
            if not self.paused:
                self.publisher.publish(std_msgs.String(str(self.curr_path)))
                self.paused = True
            else:
                self.publisher.publish(std_msgs.String(str(self.blackboard.curr_path)))
                self.paused = False
            self.goal_sent = True
        return py_trees.common.Status.RUNNING
            