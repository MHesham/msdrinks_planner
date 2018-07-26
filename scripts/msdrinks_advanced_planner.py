#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from msdrinks_planner_msgs.srv import IssueRequest
from msdrinks_planner_msgs.srv import IssueRequestRequest
from msdrinks_planner_msgs.srv import IssueRequestResponse
from msdrinks_planner_msgs.srv import GetRequestStatus
from msdrinks_planner_msgs.srv import GetRequestStatusResponse
from object_detection_msgs.msg import DetectedObject
from object_detection_msgs.msg import DetectedObjectsArray
from geometry_msgs.msg import Twist
from apriltags2_ros.msg import AprilTagDetection
from apriltags2_ros.msg import AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos):
        rospy.loginfo('going to ({},{})'.format(pos['x'], pos['y']))
        quat = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


class MsDrinksPlanner:
    STATE_IDLE = 'idle'
    STATE_REQUEST_ARRIVED = 'req-arrived'
    STATE_REACH_PROVIDER = 'reach-prov'
    STATE_REACH_REQUESTOR = 'reach-req'
    STATE_STOCK_REFILL = 'stock-refill'
    STATE_DELIVER = 'deliver'
    STATE_FULFILLED = 'fulfilled'
    STATE_REACH_STATION = 'reach-stn'

    PROVIDER_DEFAULT_ID = 0
    STATION_DEFAULT_ID = 1

    def __init__(self):
        rospy.loginfo('msdrinks planner starting...')
        self.state = self.STATE_IDLE
        self.stock = {}
        self.active_request = None
        self.detected_items = None
        self.navigator = GoToPose()
        rospy.on_shutdown(self.on_shutdown)
        rospy.Subscriber('/detected_objects/boxes', DetectedObjectsArray,
                         self.on_stock_detected_items, queue_size=1)

        self.issue_req_srv = rospy.Service(
            'IssueRequest', IssueRequest, self.on_issue_request)
        self.get_req_status_srv = rospy.Service(
            'GetRequestStatus', GetRequestStatus, self.on_get_request_status)
        self.station_pos = {'x': 1.02, 'y': -0.408}
        self.provider_pos = {'x': -2.27, 'y': -1.43}
        self.requestor_pos = [
            {'x': 2.88, 'y': -4.39}, {'x': 0.537, 'y': -6.12}]
        
        self.cmd_vel_pub = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)

        initial_cmd = Twist()
        initial_cmd.linear.x = 0.3
        initial_cmd.linear.y = 0.2
        rate = rospy.Rate(10)
        for x in range(0, 30):
            self.cmd_vel_pub.publish(initial_cmd)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

    def on_stock_detected_items(self, detected_items):
        self.detected_items = detected_items.objects
        rospy.loginfo('can see {} items'.format(len(self.detected_items)))

    def on_issue_request(self, srv_request):
        rospy.loginfo('received {} request from requestor {}'.format(
            srv_request.item_name, srv_request.requestor_id))
        if self.state != self.STATE_IDLE:
            rospy.logerr(
                'ignoring request, another request is already in progress')
            return False
        elif self.detected_items is None:
            rospy.logerr('stock object detection is not up and running')
            return False
        elif srv_request.requestor_id > len(self.requestor_pos):
            rospy.logerr('requestor ID out of range')
            return False

        self.active_request = srv_request
        self.state = self.STATE_REQUEST_ARRIVED
        return True

    def on_get_request_status(self, srv_request):
        return GetRequestStatusResponse.STATUS_INVALID

    def is_planner_ready(self):
        if (self.detected_items is None):
            return False
        return True

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.is_planner_ready() and (self.state != self.STATE_IDLE):
                self.update()
            rate.sleep()

    def update(self):
        previous_state = self.state
        assert(self.state != self.STATE_IDLE)
        if self.state == self.STATE_REQUEST_ARRIVED:
            self.update_req_arrived()
        elif self.state == self.STATE_REACH_PROVIDER:
            self.update_reach_prov()
        elif self.state == self.STATE_REACH_REQUESTOR:
            self.update_reach_req()
        elif self.state == self.STATE_REACH_STATION:
            self.update_reach_station()
        elif self.state == self.STATE_STOCK_REFILL:
            self.update_stock_refill()
        elif self.state == self.STATE_DELIVER:
            self.update_deliver()
        elif self.state == self.STATE_FULFILLED:
            self.update_fulfilled()

        if previous_state != self.state:
            rospy.loginfo(
                'state updated {} -> {}'.format(previous_state, self.state))

    def update_req_arrived(self):
        if self.is_item_in_stock():
            self.state = self.STATE_REACH_REQUESTOR
        else:
            self.state = self.STATE_REACH_PROVIDER

    def update_reach_req(self):
        success = self.navigator.goto(
            self.requestor_pos[self.active_request.requestor_id])
        if success:
            rospy.loginfo("Hooray, reached the requestor pose")
            self.state = self.STATE_DELIVER
        else:
            rospy.loginfo("The base failed to reach the requestor pose")
            self.state = self.STATE_REQUEST_ARRIVED

    def update_reach_prov(self):
        success = self.navigator.goto(self.provider_pos)
        if success:
            rospy.loginfo("Hooray, reached the provider pose")
            self.state = self.STATE_STOCK_REFILL
        else:
            rospy.loginfo("The base failed to reach the provider pose")
            self.state = self.STATE_REQUEST_ARRIVED

    def update_reach_station(self):
        success = self.navigator.goto(self.station_pos)
        if success:
            rospy.loginfo("Hooray, reached the stations pose")
            self.state = self.STATE_IDLE

    def update_stock_refill(self):
        if self.is_item_in_stock():
            self.state = self.STATE_REACH_REQUESTOR

    def update_deliver(self):
        if not self.is_item_in_stock():
            self.state = self.STATE_FULFILLED

    def update_fulfilled(self):
        self.state = self.STATE_REACH_STATION

    def is_item_in_stock(self):
        assert(self.detected_items is not None)
        for item in self.detected_items:
            if item.label.lower() == self.active_request.item_name.lower():
                return True
        return False

    def on_shutdown(self):
        #rospy.loginfo("stopping the robot base")
        # self.cmd_vel_pub.publish(Twist())
        # rospy.sleep(1)
        self.navigator.shutdown()


if __name__ == '__main__':
    rospy.init_node('msdrinks_planner', anonymous=True)
    planner = MsDrinksPlanner()
    planner.spin()
