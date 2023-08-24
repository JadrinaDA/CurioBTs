#!/usr/bin/env python

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
from extrabehaviors import Hello, Count, Random, Adder, MoveClient
from std_msgs.msg import Bool

def create_root():
    root = py_trees.composites.Parallel("Root")
    sequence = py_trees.composites.Sequence("Sequence")
    sequence2 = py_trees.composites.Sequence("Sequence")
    obsequence = py_trees.composites.Sequence("Check Obstacle")
    gh_seq = py_trees.composites.Sequence("Go Home")
    fallback = py_trees.composites.Selector("Fallback")
    fallback2 = py_trees.composites.Selector("Fallback")
    #parallel = py_trees.composites.Parallel("Parallel")
    counter = Count(name="Count")
    #hello  = Hello(name="Hello")
    random = Random(name="Random")
    #adder = Adder(name="Adder")
    moveclient = MoveClient(name="MoveRobot 1")
    moveclient2 = MoveClient(name="MoveRobot 2")
    moveclienth = MoveClient(name="MoveRobot Home")
    moveclienth2 = MoveClient(name="MoveRobot Home")

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

    pclear2bb = py_trees_ros.subscribers.ToBlackboard(
        name="PathClear2BB",
        topic_name="/pathclear",
        topic_type=Bool,
        blackboard_variables = {'pathclear': 'data'}
    )

    is_gohome_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="GoHome?",
        variable_name='gohome',
        expected_value=True
    )

    is_path_clear = py_trees.blackboard.CheckBlackboardVariable(
        name="Obstacle?",
        variable_name="pathclear",
        expected_value=True
    )

    sequence2.add_children([gohome2bb, behavior2bb, pclear2bb])
    
    #parallel.add_children([counter, random])
    sequence.add_children([moveclient, moveclient2])
    gh_seq.add_children([is_gohome_requested, moveclienth])
    obsequence.add_children([is_path_clear, moveclienth2])
    fallback2.add_children([obsequence, sequence])
    fallback.add_children([gh_seq, fallback2])
    #root.add_children([sequence, fallback])
    root.add_children([sequence2, fallback])
    return root

def shutdown(behavior_tree):
    behavior_tree.interrupt()

def main():
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