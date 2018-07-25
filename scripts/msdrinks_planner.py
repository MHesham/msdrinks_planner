#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from msdrinks_planner_msgs.msg import ItemRequest
from object_detection_msgs.msg import DetectedObject
from object_detection_msgs.msg import DetectedObjectsArray
from geometry_msgs.msg import Twist
from apriltags2_ros.msg import AprilTagDetection
from apriltags2_ros.msg import AprilTagDetectionArray


class MsDrinksPlanner:
    STATE_INIT = 'init'
    STATE_REQUEST_ARRIVED = 'req-arrived'
    STATE_FIND_REQUESTOR = 'find-req'
    STATE_FIND_PROVIDER = 'find-prov'
    STATE_REACH_PROVIDER = 'reach-prov'
    STATE_REACH_REQUESTOR = 'reach-prov'
    STATE_STOCK_REFILL = 'stock-refill'
    STATE_DELIVER = 'deliver'
    STATE_COMPLETE = 'complete'

    PROVIDER_DEFAULT_ID = 0

    def __init__(self):
        self.state = self.STATE_INIT
        self.detected_items = None
        self.active_request = None
        self.stock = {}
        self.provider_id = self.PROVIDER_DEFAULT_ID
        self.tag_detections = None
        rospy.on_shutdown(self.on_shutdown)
        rospy.Subscriber('/drinks_cmd', ItemRequest,
                         self.on_request, queue_size=1)
        rospy.Subscriber('/detected_objects/boxes', DetectedObjectsArray,
                         self.on_stock_detected_items, queue_size=1)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray,
                         self.on_tag_detection, queue_size=1)

        self.cmd_vel_pub = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)

    def on_stock_detected_items(self, detected_items):
        self.detected_items = detected_items.objects
        rospy.loginfo('can see {} items'.format(len(self.detected_items)))

    def on_tag_detection(self, tag_detections):
        self.tag_detections = tag_detections.detections
        rospy.loginfo('can see {} tags'.format(len(self.tag_detections)))

    def on_request(self, request):
        rospy.loginfo('received {} request from entity {}'.format(
            request.item_name, request.requestor_id))

        assert(self.active_request == None)
        self.active_request = request
        self.state = self.STATE_REQUEST_ARRIVED
        while self.state != self.STATE_COMPLETE:
            self.update()
        self.active_request = None
        rospy.loginfo('request fulfilled')

    def update(self):
        rospy.loginfo('current state is {}'.format(self.state))
        previous_state = self.state

        if self.state == self.STATE_REQUEST_ARRIVED:
            self.update_req_arrived()
        elif self.state == self.STATE_FIND_REQUESTOR:
            self.update_find_req()
        elif self.state == self.STATE_FIND_PROVIDER:
            self.update_find_prov()
        elif self.state == self.STATE_REACH_PROVIDER:
            self.update_reach_prov()
        elif self.state == self.STATE_REACH_REQUESTOR:
            self.update_reach_req()
        elif self.state == self.STATE_STOCK_REFILL:
            self.update_stock_refill()
        elif self.state == self.STATE_DELIVER:
            self.update_deliver()

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

    def update_reach_req(self):
        raise rospy.ROSException()

    def update_reach_prov(self):
        raise rospy.ROSException()

    def update_stock_refill(self):
        if self.is_item_in_stock():
            self.state = self.STATE_FIND_REQUESTOR

    def update_deliver(self):
        if not self.is_item_in_stock():
            self.state = self.STATE_COMPLETE

    def is_item_in_stock(self):
        for item in self.detected_items.objects:
            if item.label == self.active_request.item_name:
                return True
        return False

    def is_tag_found(self, tag_id):
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
    rospy.spin()
