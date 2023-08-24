#!/usr/bin/env python

##############################################################################
# Documentation
##############################################################################
"""
ROS node that store paths and sends each step as a goal to the MoveBAse Client to
get the robot to move.
"""
##############################################################################
# Imports
##############################################################################

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

##############################################################################
# Main Class
##############################################################################

class MoveBaseSeq():
    """
    Class that hold MoveBase Client and its paths.

    Attributes:
        points_paths (:obj:`str`): the points of the paths for easy printing
        paths (:obj:`str`): list of paths as poses
        goal_cnt (:obj:`int`): how many goals of path have been completed
        curr_path (:obj:`int`): index of the current path
        goal_s (:obj:`str`): current goal in string form
        ind (:obj:`int`): current goal in string form
    
    """
    def __init__(self):

        rospy.init_node('follow_path')
        points_seqs = rospy.get_param('follow_path/paths_points')
        self.points_paths = points_seqs
        # Get the angles from the launch file
        yaweulerangles_seqs = rospy.get_param('follow_path/paths_angles')
        # Dictionary will store each path as a list of poses
        self.paths = {}
        # Index to see where we are on the path (each path has 3 points max)
        self.goal_cnt = 0
        # Index to see on which path we are on, start with 1 because 0 is home path
        self.curr_path = 1
        # To get pose we must have orientation which we get from the angles
        quat_seqs = []
        for yawangle_seq in yaweulerangles_seqs:
            quat_seq = list()
            for yawangle in yawangle_seq:
                quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
            quat_seqs.append(quat_seq)
        ind = 0
        # Combine the point and the orientation into a pose
        for points_seq in points_seqs:
            pose_seq = list()
            n = 3
            points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
            for point in points:
                pose_seq.append(Pose(Point(*point),quat_seqs[ind][n-3]))
                n += 1
            # Store the path in our dictionary
            self.paths[ind] = pose_seq
            ind += 1
        # Goal string to send to dashboard
        self.goal_s = "(" + str(points_seqs[1][self.goal_cnt*3]) + "," + str(points_seqs[0][self.goal_cnt*3+ 1]) +")"
        # Index for creating new paths
        self.ind = ind
        # Create the MoveBase Client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Two publishers to get info to behavior tree
        self.publisher = rospy.Publisher("/status", String, queue_size=10, latch=True)
        self.state_pub = rospy.Publisher("/state", String, queue_size=3, latch=True)
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
        self.state_pub.publish(String("1"))

    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        #rospy.loginfo(str(feedback))
        # Publish the state so that our node on the tree can return the correct status
        state = self.client.get_state()
        #self.state_pub.publish(String(str(state)))

    def done_cb(self, status, result):
        # When a goal is reached...
        # We increase our goal count
        self.goal_cnt += 1
        # Get our current path
        self.pose_seq = self.paths[self.curr_path]
        # If the goal was canceled print out and decrease the goal count (pause)
        if status == 2:
            self.state_pub.publish(String(str(status)))
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")
            self.goal_cnt -= 1
        # If goal was reached
        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            # Publish to dashboard
            self.state_pub.publish(String(str(status)))
            self.publisher.publish(String("Arrived at " + str(self.goal_s)))
            #rospy.spin()
            # If there are still goals in our path send the next goal
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
            # If not you finished the path and have to stop and wait for the next command
            else:
                rospy.loginfo("Final goal pose reached!")
                self.goal_cnt = 0
                #rospy.signal_shutdown("Final goal pose reached!")
                #return
        
        # Other status usually do not happen
        
        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            self.state_pub.publish(String(str(status)))
            #rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            #return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def switch_path(self, data):
        """
        Callback function that switched the current path based on the info received from
        the topic.

        Attributes:
            data (:obj:`msg`): message containing the index of chosen path
        
        """
        # If we had gone home and now are going to another path reset the goal count
        if self.curr_path == 0 and int(data.data) != 0:
            self.goal_cnt = 0
        # Set the new path
        self.curr_path = int(data.data)
        # If there is no path with that index dont do anything and send a message
        if self.curr_path not in self.paths.keys() and self.curr_path > 0:
            rospy.loginfo("No such path")
            self.curr_path = 0
            return
        # If we were told to pause cancel the goal
        if self.curr_path == -1:
            self.client.cancel_goal()
            self.curr_path = 1
        else:
            # Get the current path
            self.pose_seq = self.paths[self.curr_path]
            # If you are sent home reset goal count
            if self.curr_path == 0:
                self.goal_cnt = 0
            # If not go to the equivalent point on the chosen path
            if self.goal_cnt< len(self.pose_seq):
                    self.pose_seq = self.paths[self.curr_path]
                    next_goal = MoveBaseGoal()
                    next_goal.target_pose.header.frame_id = "map"
                    next_goal.target_pose.header.stamp = rospy.Time.now()
                    next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                    rospy.loginfo("Switching to path " + data.data)
                    self.goal_s = "(" + str(self.points_paths[self.curr_path][self.goal_cnt*3]) + "," + str(self.points_paths[self.curr_path][self.goal_cnt*3+ 1]) +")"
                    rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                    self.state_pub.publish(String("1"))
                    rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                    # Send new goal to dashboard
                    self.publisher.publish(String("Moving to " + str(self.goal_s)))
                    self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)

    
    def movebase_client(self):
        # We'll have a subscriber listening for a switch in path
        rospy.Subscriber("move_base/switch_path", String, self.switch_path)
        # This topic is to add new paths from RViz
        rospy.Subscriber("/clicked_point", PointStamped, self.add_point)
        
        # Start off with our default paths
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.paths[self.curr_path][self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        self.publisher.publish(String("Moving to " + str(self.goal_s)))
        rospy.loginfo(str(self.paths[self.curr_path][self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

    def add_point(self, data):
        # Add a new point to the last path on our list
        # Get point from the data
        point = [round(data.point.x,1), round(data.point.y,1), 0]
        # Angle is always 0 because we dont care
        quat = Quaternion(*(quaternion_from_euler(0, 0, 0*math.pi/180, axes='sxyz')))
        # If we are starting a new path we need to create the empty lists
        if len(self.paths.keys()) < self.ind + 1:
            self.paths[self.ind] = []
            self.points_paths.append([])
        # Add first the point to the list of poses
        self.paths[self.ind].append(Pose(Point(*point),quat))
        # Then add it to the list of points
        for p in point:
            self.points_paths[self.ind].append(p)
        # If our path is complete (it has three points)
        if len(self.paths[self.ind]) == 3:
            # Increase index to start new path and print out all paths
            self.ind += 1
            rospy.loginfo(self.points_paths)


##############################################################################
# Main
##############################################################################
if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

