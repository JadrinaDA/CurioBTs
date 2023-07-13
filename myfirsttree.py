#!/usr/bin/env python

##############################################################################
# Documentation
##############################################################################
"""
Basic py_tree_ros example that just has robot between two points and can be told
to go home.
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
from extrabehaviors import MoveClient
from std_msgs.msg import Bool

##############################################################################
# Behaviours
##############################################################################


def create_root():
    """
    Creates the different behaviors and composites and joins them together 
    in tree structure.

    root --> sequence --> gohome2bb, behaviorbb
         --> fallback --> go home  --> gohome?, moverobothome
                      --> sequence --> moverobot1, moverobot2  
    """

    root = py_trees.composites.Parallel("Root")
    sequence = py_trees.composites.Sequence("Sequence")
    sequence2 = py_trees.composites.Sequence("Sequence")
    gh_seq = py_trees.composites.Sequence("Go Home")
    fallback = py_trees.composites.Selector("Fallback")
    moveclient = MoveClient(name="MoveRobot 1")
    moveclient2 = MoveClient(name="MoveRobot 2")
    moveclienth = MoveClient(name="MoveRobot Home")

    # Gets message about what robot is doing to bb
    behavior2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Behavior2BB",
        topic_name="/dashboard/behavior",
        variable_name="curr_behavior"
    )

    # Gets message to gohome to bb, if true robot must go home
    gohome2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="GoHome2BB",
        topic_name="/dashboard/gohome",
        variable_name="gohome"
    )

    # If variable gohome is true robot is sent home
    is_gohome_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="GoHome?",
        variable_name='gohome',
        expected_value=True
    )

    sequence2.add_children([gohome2bb, behavior2bb])
    
    sequence.add_children([moveclient, moveclient2])
    gh_seq.add_children([is_gohome_requested, moveclienth])
    fallback.add_children([gh_seq, sequence])
    root.add_children([sequence2, fallback])
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