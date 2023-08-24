#!/usr/bin/env python
##############################################################################
# Documentation
##############################################################################
"""
Behavior tree that controls path following robot, which can switch paths, go home,
pause, detect when it is stuck and ask for help. It is also controlled with the
help of a dashboard where the user can input the chosen path and trigger other
behaviors.
"""
##############################################################################
# Imports
##############################################################################
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
from mybehaviors import FollowPath, ChangePath, GoHome, Pause, Ask4Help, TriggerRB
from std_msgs.msg import String, Bool

##############################################################################
# Tree structure
##############################################################################

def create_root():
    """
    Creates the different behaviors and composites and joins them together 
    in tree structure.

    root --> sequence --> gohome2bb, behaviorbb, pause2bb, pathclear2bb
         --> sequence --> state2bb
         --> fallback --> pause
                      --> go home  --> gohome?, gohome
                      --> fallback --> sequence --> inverter --> nocommand
                                                --> changepath
                                   --> sequence --> pathnotclear, ask4help
                                   --> followpath
         --> command2bb  
    
    Basically we are updating the data as often as possible with our topmost 
    sequences. Then we have a fallback where the highest priority is the pause
    behavior. If the button is pressed the robot must stop everything until told
    to resume. Then if not paused it checks if it has to go home. If not paused
    or told to go home, then the robot will follow the chosen path. First it will
    check that it has not received a command to change path and that the path is clear.
    If none of these happen then it will just follow the path. If it is unable to follow
    the path and for some reason aborts the goal the robot will simply remain idle until
    it receives another command.
    
    """

    root = py_trees.composites.Parallel("Root")
    sequence = py_trees.composites.Sequence("Sequence")
    sequence2 = py_trees.composites.Sequence("Sequence")
    gh_seq = py_trees.composites.Sequence("Go Home")
    fallback = py_trees.composites.Selector("Fallback")
    fallback2 = py_trees.composites.Selector("Fallback")
    changes = py_trees.composites.Sequence("Change Path?")
    changepath = ChangePath(name="Change Path")
    followpath = FollowPath(name="Follow Path")
    gohome = GoHome(name="Drive Home")
    ask4help = Ask4Help(name="Ask4Help")
    checkpath = py_trees.composites.Sequence("Check Path")
    idle = py_trees.behaviours.Running(name="Idle")

    behavior2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Behavior2BB",
        topic_name="/dashboard/behavior",
        variable_name="curr_behavior"
    )

    gohome2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="GoHome2BB",
        topic_name="/dashboard/gohome",
        variable_name="gohome"
    )

    pause2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Pause2BB",
        topic_name="/dashboard/pause",
        variable_name="pause"
    )

    is_gohome_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="GoHome?",
        variable_name='gohome',
        expected_value=True
    )

    pathclear2bb = py_trees_ros.subscribers.ToBlackboard(
        name="PathClearBB",
        topic_name="/pathclear",
        topic_type=Bool,
        blackboard_variables = {'pathclear': 'data'}
    )

    # The name on this one is confusing really its true if the path is not clear 
    is_path_clear = py_trees.blackboard.CheckBlackboardVariable(
        name="PathNotClear?",
        variable_name='pathclear',
        expected_value=True
    )

    state2bb = py_trees_ros.subscribers.ToBlackboard(
        name="State2BB",
        topic_name="/state",
        topic_type=String,
        blackboard_variables = {'state': 'data'}
    )

    command2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Comm2BB",
        topic_name="/dashboard/command",
        topic_type=String,
        blackboard_variables = {'command': 'data'}
    )

    no_command = py_trees.blackboard.CheckBlackboardVariable(
        name="NoCommand?",
        variable_name='command',
        expected_value="None"
    )

    decorator = py_trees.decorators.Inverter(name="Inverter", child=no_command)
    pause = Pause("Pause")


    sequence2.add_children([gohome2bb, behavior2bb, pause2bb, pathclear2bb])
    # The update rate of this topic is different so it must be separated to not block
    # the others
    sequence.add_children([state2bb])
    
    fallback2.add_children([changes, checkpath, followpath, idle])
    changes.add_children([decorator, changepath])
    checkpath.add_children([is_path_clear, ask4help])
    #checkpath.add_children([is_path_clear, triggerb, ask4help])
    gh_seq.add_children([is_gohome_requested, gohome])
    fallback.add_children([pause, gh_seq, fallback2])
    # Command2bb is only updated when clicked so it must be separated to not block
    root.add_children([sequence2, sequence, fallback, command2bb])
    return root

def shutdown(behavior_tree):
    behavior_tree.interrupt()


##############################################################################
# Main
##############################################################################
def main():
    """
    Entry point for the script Creates ros node and tree before executing it.
    """
    rospy.init_node("tree")
    root = create_root()
    behavior_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behavior_tree))
    if not behavior_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting")
        sys.exit(1)
    behavior_tree.tick_tock(500)

if __name__ == '__main__':
    main()