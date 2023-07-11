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

class Hello(py_trees.behaviour.Behaviour):
    """
    Behavior just prints hello world to console.
    """
    def __init__(self, name):
        super(Hello, self).__init__(name=name)

    def setup(self, timeout):
        self.message = "Hello World"
        return True

    def update(self):
        print(self.message)
        return py_trees.common.Status.SUCCESS

class Count(py_trees.behaviour.Behaviour):
    """
    A counting behaviour that updates its status at each tick depending on
    the value of the counter. The status will move through the states in order -
    :data:`~py_trees.common.Status.FAILURE`, :data:`~py_trees.common.Status.RUNNING`,
    :data:`~py_trees.common.Status.SUCCESS`.

    This behaviour is useful for simple testing and demo scenarios.

    Args:
        name (:obj:`str`): name of the behaviour
        fail_until (:obj:`int`): set status to :data:`~py_trees.common.Status.FAILURE` until the counter reaches this value
        running_until (:obj:`int`): set status to :data:`~py_trees.common.Status.RUNNING` until the counter reaches this value
        success_until (:obj:`int`): set status to :data:`~py_trees.common.Status.SUCCESS` until the counter reaches this value
        reset (:obj:`bool`): whenever invalidated (usually by a sequence reinitialising, or higher priority interrupting)

    Attributes:
        count (:obj:`int`): a simple counter which increments every tick
    """
    def __init__(self, name="Count", fail_until=3, running_until=5, success_until=6, reset=True):
        super(Count, self).__init__(name)
        self.count = 0
        self.fail_until = fail_until
        self.running_until = running_until
        self.success_until = success_until
        self.number_count_resets = 0
        self.number_updated = 0
        self.reset = reset

    def setup(self, timeout):
        return True

    def terminate(self, new_status):
        self.logger.debug("%s.terminate(%s->%s)" % (self.__class__.__name__, self.status, new_status))
        # reset only if update got us into an invalid state
        print(self.feedback_message)
        if new_status == py_trees.common.Status.INVALID and self.reset:
            self.count = 0
            self.number_count_resets += 1
        self.feedback_message = ""

    def update(self):
        self.number_updated += 1
        self.count += 1
        if self.count <= self.running_until:
            self.logger.debug("%s.update()[%s: running]" % (self.__class__.__name__, self.count))
            self.feedback_message = "running"
            return py_trees.common.Status.RUNNING
        elif self.count <= self.success_until:
            self.logger.debug("%s.update()[%s: success]" % (self.__class__.__name__, self.count))
            self.feedback_message = "success"
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("%s.update()[%s: failure]" % (self.__class__.__name__, self.count))
            self.feedback_message = "failing forever more"
            return py_trees.common.Status.FAILURE

    def __repr__(self):
        """
        Simple string representation of the object.

        Returns:
            :obj:`str`: string representation
        """
        s = "%s\n" % self.name
        s += "  Status : %s\n" % self.status
        s += "  Count  : %s\n" % self.count
        s += "  Resets : %s\n" % self.number_count_resets
        s += "  Updates: %s\n" % self.number_updated
        return s

class Random(py_trees.behaviour.Behaviour):
    """
    Behavior generate a number and returns success if it is even.
    """
    def __init__(self, name):
        super(Random, self).__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.number = 0

    def setup(self, timeout):
        self.number = 0
        return True

    def update(self):
        self.number = random.randint(1, 10)
        print(self.number)
        if self.number % 2 == 0:
            self.blackboard.number = self.number
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        

class Adder(py_trees.behaviour.Behaviour):
    """
    Behavior adds number it gets from blackboard.
    """
    def __init__(self, name):
        super(Adder, self).__init__(name=name)

    def setup(self, timeout):
        self.number = 0
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def update(self):
        self.number += self.blackboard.number
        print(self.number)
        return py_trees.common.Status.SUCCESS
    
class MoveClient(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super(MoveClient, self).__init__(name=name)
        if (name.split(" ")[1] == "Home"):
            self.num = 3
        else:
            self.num = int(name.split(" ")[1]) - 1

        # Prepare list of goals
        points_seq = [1,1,0,1,5,0,3,3,0,0,0,0]
        self.goal_s = "(" + str(points_seq[self.num*3]) + "," + str(points_seq[self.num*3+ 1]) +")"
        yaweulerangles_seq = [90,0,180,0]
        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0
        self.dist = [0,10,10]
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

    def setup(self, timeout):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.num]
        self.goal = goal
        self.goal_sent = False
        self.publisher = rospy.Publisher("/status", std_msgs.String, queue_size=10, latch=True)
        return True
    
    def initialise(self):
        self.goal_sent = False
    
    def update(self):
        if (not self.goal_sent):
            self.client.send_goal(self.goal)
            self.goal_sent = True
        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            self.publisher.publish(std_msgs.String("Arrived at " + str(self.goal_s)))
            return py_trees.common.Status.SUCCESS
        elif state == GoalStatus.ACTIVE or state == GoalStatus.PENDING:
            self.publisher.publish(std_msgs.String("Moving to " + str(self.goal_s)))
            return py_trees.common.Status.RUNNING
        else:
            print(state)
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        state = self.client.get_state()
        if state == GoalStatus.ACTIVE:
            self.publisher.publish(std_msgs.String("Goal was preempted"))