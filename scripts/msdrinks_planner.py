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


class MsDrinksPlanner:
    STATE_IDLE = 'idle'
    STATE_REQUEST_ARRIVED = 'req-arrived'
    STATE_FIND_REQUESTOR = 'find-req'
    STATE_FIND_PROVIDER = 'find-prov'
    STATE_FIND_STATION = 'find-stn'
    STATE_REACH_PROVIDER = 'reach-prov'
    STATE_REACH_REQUESTOR = 'reach-prov'
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
        self.provider_id = self.PROVIDER_DEFAULT_ID
        self.station_id = self.PROVIDER_DEFAULT_ID
        self.active_request = None
        self.detected_items = None
        self.tag_detections = None
        rospy.on_shutdown(self.on_shutdown)
        rospy.Subscriber('/detected_objects/boxes', DetectedObjectsArray,
                         self.on_stock_detected_items, queue_size=1)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray,
                         self.on_tag_detection, queue_size=1)

        self.cmd_vel_pub = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)

        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        # allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        self.issue_req_srv = rospy.Service(
            'IssueRequest', IssueRequest, self.on_issue_request)
        self.get_req_status_srv = rospy.Service(
            'GetRequestStatus', GetRequestStatus, self.on_get_request_status)

    def on_stock_detected_items(self, detected_items):
        self.detected_items = detected_items.objects
        rospy.loginfo('can see {} items'.format(len(self.detected_items)))

    def on_tag_detection(self, tag_detections):
        self.tag_detections = tag_detections.detections
        rospy.loginfo('can see {} tags'.format(len(self.tag_detections)))

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
        elif self.tag_detections is None:
            rospy.logerr('tag detection is not up and running')
            return False

        self.active_request = srv_request
        self.state = self.STATE_REQUEST_ARRIVED
        return True

    def on_get_request_status(self, srv_request):
        return GetRequestStatusResponse.STATUS_INVALID

    def is_planner_ready(self):
        if (self.detected_items is None) or (self.tag_detections is None):
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
        elif self.state == self.STATE_FIND_REQUESTOR:
            self.update_find_req()
        elif self.state == self.STATE_FIND_PROVIDER:
            self.update_find_prov()
        elif self.state == self.STATE_FIND_STATION:
            self.update_find_station()
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
            self.state = self.STATE_FIND_REQUESTOR
        else:
            self.state = self.STATE_FIND_PROVIDER

    def find_tag_by_id(self, tag_id):
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.3
        rate = rospy.Rate(10)
        while not self.is_tag_found(tag_id):
            self.cmd_vel_pub.publish(rotate_cmd)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

    def update_find_req(self):
        self.find_tag_by_id(self.active_request.requestor_id)
        self.state = self.STATE_REACH_REQUESTOR

    def update_find_prov(self):
        self.find_tag_by_id(self.provider_id)
        self.state = self.STATE_REACH_REQUESTOR

    def update_find_station(self):
        self.find_tag_by_id(self.station_id)
        self.state = self.STATE_REACH_STATION

    def update_reach_req(self):
        pose = None
        for tag in self.tag_detections:
            if tag[0].id == self.active_request.requestor_id:
                pose = tag[0].pose.pose
                break
        if pose is None:
            self.state = self.STATE_REQUEST_ARRIVED
        else:
            if self.go_to(pose):
                self.state = self.STATE_DELIVER
            else:
                self.state = self.STATE_REQUEST_ARRIVED

    def update_reach_prov(self):
        pose = None
        for tag in self.tag_detections:
            if tag[0].id == self.provider_id:
                pose = tag[0].pose.pose
                break
        if pose is None:
            self.state = self.STATE_REQUEST_ARRIVED
        else:
            if self.go_to(pose):
                self.state = self.STATE_STOCK_REFILL
            else:
                self.state = self.STATE_REQUEST_ARRIVED

    def update_reach_station(self):
        pose = None
        for tag in self.tag_detections:
            if tag[0].id == self.station_id:
                pose = tag[0].pose.pose
                break
        if pose is not None:
            if self.go_to(pose):
                self.state = self.STATE_IDLE

    def update_stock_refill(self):
        if self.is_item_in_stock():
            self.state = self.STATE_FIND_REQUESTOR

    def update_deliver(self):
        if not self.is_item_in_stock():
            self.state = self.STATE_FULFILLED

    def update_fulfilled(self):
        self.state = self.STATE_IDLE

    def go_to(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        # start moving
        self.move_base.send_goal(goal)

        # allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.logwarn(
                "The base failed to move forward 3 meters for some reason")
            return False
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(
                    "Hooray, the base moved arrived at {}".format(pos))
            return True

    def is_item_in_stock(self):
        assert(self.detected_items is not None)
        for item in self.detected_items:
            if item.label == self.active_request.item_name:
                return True
        return False

    def is_tag_found(self, tag_id):
        assert(self.detected_items is not None)
        for tag in self.tag_detections:
            if tag[0].id == tag_id:
                return True
        return False

    def on_shutdown(self):
        rospy.loginfo("stopping the robot base")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('msdrinks_planner', anonymous=True)
    planner = MsDrinksPlanner()
    planner.spin()
