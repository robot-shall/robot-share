#!/usr/bin/env python3

'''
test sample
"{position: {x: 1.536250, y: 0.580879, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
'''

import rospy
import actionlib

from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class NavModule():
    def __init__(self) -> None:
        self.goal = MoveBaseGoal()
        self.have_goal = False
        self.move_action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def set_goal(self, pose): 
        '''
        设置目标
        传入参数:
        - pose: geometry_msgs.msg.Pose
        功能:
        设置导航的目标
        '''
        rospy.loginfo("get a goal, x: {}, y:{}, z:{}".format(pose.position.x, pose.position.y, pose.position.z))
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        # self.goal.target_pose.pose.position.x = x
        # self.goal.target_pose.pose.position.y = y
        # self.goal.target_pose.pose.orientation.w = w
        self.goal.target_pose.pose = pose

        self.have_goal = True

        self.move()

    def move(self, secs=60): #移动到目标点
        self.move_action_client.wait_for_server(rospy.Duration(60))

        if not self.have_goal:
            rospy.logerr("please set a goal first")
            return 
        
        rospy.loginfo("starting moving")
        self.have_goal = False

        self.move_action_client.send_goal(self.goal)
        finish_res = self.move_action_client.wait_for_result(rospy.Duration(secs))

        if not finish_res:
            self.move_action_client.cancel_goal()
            rospy.loginfo("not finished")
        else:
            code = self.move_action_client.get_state()
            if code == GoalStatus.SUCCEEDED:
                rospy.loginfo("goal succeed")
                self.move_ok()
            else:
                rospy.loginfo("something is wrong, code is {}".format(code))

    def move_ok(self): # 在这里加入到达目标之后的逻辑
        #TODO
        pass

    def run(self):
        rospy.init_node("nav_wrapper", anonymous=False)
        rospy.Subscriber("/wrapper_set_goal", Pose, callback=self.set_goal) # 启动一个subscriber用于监听 /wrapper_set_goal 用于接收导航目标位置信息

        rospy.spin()


def on_shutdown():
    rospy.loginfo("exiting ...")

if __name__ == "__main__":
    app = NavModule()
    app.run()
    rospy.on_shutdown(on_shutdown)
    