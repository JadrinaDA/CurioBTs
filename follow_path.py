#!/usr/bin/env python
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('follow_path')
        points_seqs = rospy.get_param('follow_path/paths_points')
        self.points_paths = points_seqs
        yaweulerangles_seqs = rospy.get_param('follow_path/paths_angles')
        self.paths = {}
        self.goal_cnt = 0
        self.curr_path = 1
        quat_seqs = []
        for yawangle_seq in yaweulerangles_seqs:
            quat_seq = list()
            for yawangle in yawangle_seq:
                quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
            quat_seqs.append(quat_seq)
        ind = 0
        for points_seq in points_seqs:
            pose_seq = list()
            n = 3
            points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
            for point in points:
                pose_seq.append(Pose(Point(*point),quat_seqs[ind][n-3]))
                n += 1
            self.paths[ind] = pose_seq
            ind += 1
        self.goal_s = "(" + str(points_seqs[1][self.goal_cnt*3]) + "," + str(points_seqs[0][self.goal_cnt*3+ 1]) +")"
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.publisher = rospy.Publisher("/status", String, queue_size=10, latch=True)
        self.state_pub = rospy.Publisher("/state", String, queue_size=10, latch=True)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        #rospy.loginfo(str(feedback))
        state = self.client.get_state()
        self.state_pub.publish(String(str(state)))
        #rospy.loginfo(type(feedback))

    def done_cb(self, status, result):
        self.goal_cnt += 1
        self.pose_seq = self.paths[self.curr_path]
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            self.publisher.publish(String("Arrived at " + str(self.goal_s)))
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                self.goal_s = "(" + str(self.points_paths[self.curr_path][self.goal_cnt*3]) + "," + str(self.points_paths[self.curr_path][self.goal_cnt*3+ 1]) +")"
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.publisher.publish(String("Moving to " + str(self.goal_s)))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                #rospy.signal_shutdown("Final goal pose reached!")
                #return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def switch_path(self, data):
        self.pose_seq = self.paths[self.curr_path]
        self.curr_path = int(data.data)
        if self.curr_path == -1:
            self.client.cancel_goal()
            self.curr_path = 1
        else:
            if self.curr_path == 0:
                self.goal_cnt = 0
            if self.goal_cnt< len(self.pose_seq):
                    self.pose_seq = self.paths[self.curr_path]
                    next_goal = MoveBaseGoal()
                    next_goal.target_pose.header.frame_id = "map"
                    next_goal.target_pose.header.stamp = rospy.Time.now()
                    next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                    rospy.loginfo("Switching to path " + data.data)
                    self.goal_s = "(" + str(self.points_paths[self.curr_path][self.goal_cnt*3]) + " , " + str(self.points_paths[self.curr_path][self.goal_cnt*3+ 1]) +")"
                    rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                    rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                    self.publisher.publish(String("Moving to " + str(self.goal_s)))
                    self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)

    
    def movebase_client(self):
        # We'll have a subscriber listening for a switch in path
        rospy.Subscriber("move_base/switch_path", String, self.switch_path)
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.paths[self.curr_path][self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        self.publisher.publish(String("Moving to " + str(self.goal_s)))
        rospy.loginfo(str(self.paths[self.curr_path][self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

