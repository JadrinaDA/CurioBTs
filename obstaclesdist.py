#!/usr/bin/env python

##############################################################################
# Documentation
##############################################################################
"""
ROS node that listens for obstacle information and the position of the robot
and then publishes if there is an obstacle in the path based on the distances
calculated.
"""
##############################################################################
# Imports
##############################################################################


import rospy
from std_msgs.msg import Bool
from obstacle_detector.msg import Obstacles, SegmentObstacle
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path
import math

# Publisher to signal to behavior tree is path is clear or not
pub = rospy.Publisher('/pathclear', Bool, queue_size=10)
# Boolean to toggle
obstacle_in_front = False
# Global variable to store robot position
pos = (0,0)
# Global variable to store next point on local path
next = None


# Callback function called when obstacle data is received
def callback(data):
    global obstacle_in_front
    global pos
    global next

    segments = data.segments
    min = 1000
    # For each segment detected calculate the middle point, then
    # calculate the distance between the middle point to robot and the angle
    for segment in segments:
        x = (segment.first_point.x + segment.last_point.x) / 2
        y = (segment.first_point.y + segment.last_point.y) / 2
        dist = math.sqrt((x-pos[0])**2 + (y-pos[1])**2)
        if next:
            result = math.atan2(y - pos[1], x - pos[0]) - math.atan2(next.y - pos[1], next.x - pos[0])
        else:
            result = 10
        rospy.loginfo(str(result)+"x")
        rospy.loginfo(dist)
        # If distance is lower than our current minimum and angle is on path save it
        if dist < min and (abs(result) < math.pi /3):
            min = dist
    circles = data.circles
    # Same process for circles
    for circle in circles:
        x = circle.center.x
        y = circle.center.y
        dist = math.sqrt((x-pos[0])**2 + (y-pos[1])**2)
        if next:
            result = math.atan2(y - pos[1], x - pos[0]) - math.atan2(next.y - pos[1], next.x - pos[0])
        else:
            result = 10
        rospy.loginfo(str(result)+"x")
        rospy.loginfo(dist)
        if dist < min and (abs(result) < math.pi /6):
            min = dist
    obstacle_in_front = False
    
    # We set a threshold to see if there is an obstacle in our path
    if min < 1.5:
        obstacle_in_front = True
    rospy.loginfo(obstacle_in_front)
    

def callbackg(data):
    # Callback when we receive robot footprint, transform into position
    global pos
    x = (data.polygon.points[0].x + data.polygon.points[1].x) / 2
    y = (data.polygon.points[0].y + data.polygon.points[1].y) / 2
    pos = (x, y)

def get_next(data):
    # Callback when we receive path, take the first point on the path
    global next
    next = data.poses[0].pose.position
    
def listener():
    global obstacle_in_front

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    # Set up our three subscribers and their callbacks
    rospy.Subscriber("/raw_obstacles", Obstacles, callback)
    rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, callbackg)
    rospy.Subscriber("/move_base/TrajectoryPlannerROS/local_plan", Path, get_next)
    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        pub.publish(obstacle_in_front)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
