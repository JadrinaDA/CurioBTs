#!/usr/bin/env python

##############################################################################
# Documentation
##############################################################################
"""
Slightly more complicated py_tree_ros example that has two possible paths, the robot can
change between the two paths and can be told to go home.
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
from mybehaviors import MoveClient
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
                      --> sequence --> fallback --> pathclear?, changepath
                                   --> followpath  
    """
    root = py_trees.composites.Parallel("Root")
    sequence2 = py_trees.composites.Sequence("Sequence")
    mainseq= py_trees.composites.Sequence("Sequence")
    gh_seq = py_trees.composites.Sequence("Go Home")
    fallback = py_trees.composites.Selector("Fallback")
    fallback2 = py_trees.composites.Selector("Fallback")
    moveclient2 = MoveClient(name="Change 1 Path")
    moveclient3 = MoveClient(name="Follow 2 Path")
    moveclienth = MoveClient(name="MoveRobot Home")

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

    is_gohome_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="GoHome?",
        variable_name='gohome',
        expected_value=True
    )

    is_path_clear = py_trees.blackboard.CheckBlackboardVariable(
        name="PathClear?",
        variable_name='pathclear',
        expected_value=True
    )

    sequence2.add_children([gohome2bb, behavior2bb])
    
    fallback2.add_children([is_path_clear, moveclient2])
    mainseq.add_children([fallback2, moveclient3])
    gh_seq.add_children([is_gohome_requested, moveclienth])
    fallback.add_children([gh_seq, mainseq])
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