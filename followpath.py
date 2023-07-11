#!/usr/bin/env python

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
from mybehaviors import FollowPath, ChangePath, GoHome, Pause
from std_msgs.msg import String

def create_root():
    root = py_trees.composites.Parallel("Root")
    sequence = py_trees.composites.Sequence("Sequence")
    sequence2 = py_trees.composites.Sequence("Sequence")
    mainseq= py_trees.composites.Sequence("Sequence")
    gh_seq = py_trees.composites.Sequence("Go Home")
    fallback = py_trees.composites.Selector("Fallback")
    fallback2 = py_trees.composites.Selector("Fallback")
    changes = py_trees.composites.Selector("Change Path?")
    changepath = ChangePath(name="Change Path")
    followpath = FollowPath(name="Follow Path")
    gohome = GoHome(name="Drive Home")

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
        topic_type=String,
        blackboard_variables = {'pathclear': 'data'}
    )

    is_path_clear = py_trees.blackboard.CheckBlackboardVariable(
        name="PathNotClear?",
        variable_name='pathclear',
        expected_value="False"
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


    sequence2.add_children([gohome2bb, behavior2bb, pause2bb])
    sequence.add_children([state2bb, pathclear2bb])
    
    fallback2.add_children([mainseq, followpath])
    changes.add_children([is_path_clear, decorator])
    mainseq.add_children([changes, changepath])
    gh_seq.add_children([is_gohome_requested, gohome])
    fallback.add_children([pause, gh_seq, fallback2])
    root.add_children([sequence2, sequence, fallback, command2bb])
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