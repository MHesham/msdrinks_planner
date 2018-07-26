#!/usr/bin/env python
import rospy
from msdrinks_planner_msgs.srv import IssueRequest
from msdrinks_planner_msgs.srv import GetRequestStatus

REQUESTOR_ID = 1


def issue_request(item_name):
    rospy.loginfo('requestor {} sending request for {}'.format(
        REQUESTOR_ID, item_name))
    try:
        rospy.wait_for_service('IssueRequest', 5.0)
        if issue_request_srv(REQUESTOR_ID, item_name).status:
            rospy.loginfo('request issued, pending...')
        else:
            rospy.logerr('request issuing failed')
    except rospy.ServiceException, e:
        rospy.logerr(e)


def get_request_status():
    try:
        rospy.loginfo('querying last request status')
        rospy.wait_for_service('GetRequestStatus', 5.0)
        status = get_request_status_srv(REQUESTOR_ID).status
        rospy.loginfo('request status:{}'.format(status))
    except rospy.ServiceException, e:
        rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('msdrinks_planner_cli', anonymous=True)
    issue_request_srv = rospy.ServiceProxy('IssueRequest', IssueRequest)
    get_request_status_srv = rospy.ServiceProxy(
        'GetRequestStatus', GetRequestStatus)

    try:
        while(1):
            item_name = raw_input('>').lower()
            if item_name == 'get':
                get_request_status()
            elif item_name == 'cancel':
                rospy.logerr('cancel request is not supported')
            else:
                issue_request(item_name)
    except rospy.ROSException, e:
        rospy.logerr(e)
    finally:
        rospy.sleep(1)
