#!/usr/bin/env python
import rospy
from msdrinks_planner_msgs.srv import IssueRequest
from msdrinks_planner_msgs.srv import GetRequestStatus
from apriltags2_ros.msg import AprilTagDetection
from apriltags2_ros.msg import AprilTagDetectionArray
from math import sqrt

INVALID_REQUESTOR_ID = -1

def get_requestor_tag_id():

    tag_detections = rospy.wait_for_message(
        '/tag_detections', AprilTagDetectionArray, 3.0)

    if len(tag_detections.detections) == 0:
        return INVALID_REQUESTOR_ID

    rospy.loginfo('can see {} tags'.format(len(tag_detections.detections)))
    min_dist = 9999.0
    min_tag = None
    for tag in tag_detections.detections:
        tag_pos = tag.pose.pose.pose.position
        dist = sqrt(tag_pos.x * tag_pos.x + tag_pos.y *
                    tag_pos.y + tag_pos.z * tag_pos.z)
        if dist < min_dist:
            min_dist = dist
            min_tag = tag
        rospy.loginfo('id:{} dist:{}'.format(tag.id[0], dist))
    return min_tag.id[0]


def wait_for_requestor_id():
    for i in range(1, 4):
        requestor_id = get_requestor_tag_id()
        if requestor_id == INVALID_REQUESTOR_ID:
            rospy.logwarn(
                'no tag id present to camera, please show your tag id')
            rospy.sleep(i * 1.0)
        else:
            return requestor_id
    rospy.logwarn('timeout waiting for requestor tag id to show up')
    return INVALID_REQUESTOR_ID

def issue_request(item_name):

    requestor_id = wait_for_requestor_id()
    if requestor_id == INVALID_REQUESTOR_ID:
        return

    rospy.loginfo('requestor {} sending request for {}'.format(
        requestor_id, item_name))
    try:
        rospy.wait_for_service('IssueRequest', 5.0)
        if issue_request_srv(requestor_id, item_name).status:
            rospy.loginfo('request issued, pending...')
        else:
            rospy.logerr('request issuing failed')
    except rospy.ServiceException, e:
        rospy.logerr(e)


def get_request_status():

    requestor_id = wait_for_requestor_id()
    if requestor_id == INVALID_REQUESTOR_ID:
        return

    try:
        rospy.loginfo('querying last request status')
        rospy.wait_for_service('GetRequestStatus', 5.0)
        status = get_request_status_srv(requestor_id).status
        rospy.loginfo('request status:{}'.format(status))
    except rospy.ServiceException, e:
        rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('msdrinks_planner_cli', anonymous=True)
    issue_request_srv = rospy.ServiceProxy('IssueRequest', IssueRequest)
    get_request_status_srv = rospy.ServiceProxy(
        'GetRequestStatus', GetRequestStatus)

    try:
        while not rospy.is_shutdown():
            item_name = raw_input('>').lower()
            if item_name == 'get':
                get_request_status()
            elif item_name == 'cancel':
                rospy.logerr('cancel request is not supported')
            else:
                issue_request(item_name)
    except rospy.ROSInterruptException:
        rospy.loginfo('exiting cli')
