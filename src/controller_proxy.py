#!/usr/bin/env python

import rospy
import json
import tornado.websocket
import tornado.ioloop
import tornado.web

from rover1.msg import input_arm
from rover1.msg import input_drive


class ControllerHandler(tornado.websocket.WebSocketHandler):
    def on_close(self):
        print("Disconnecting")

    def open(self):
        print("Connecting")

    def on_message(self, data):
        global arm_publisher
        global drive_publisher
        json_msg = json.loads(data)
        print(json_msg)
        if json_msg["type"] == "drive":
            # do the thing
            msg = input_drive()
            msg.left = json_msg["left"]
            msg.right = json_msg["right"]
            rospy.loginfo("Drive: left = [%lf]", msg.left)
            rospy.loginfo("Drive: right = [%lf]", msg.right)
            drive_publisher.publish(msg)
        elif json_msg["type"] == "arm":
            # do the thing
            msg = input_arm()
            msg.base = json_msg["base"]
            msg.shoulder = json_msg["shoulder"]
            msg.elbow = json_msg["elbow"]
            msg.wrist = json_msg["wrist"]
            msg.fingers = json_msg["fingers"]
            rospy.loginfo("Arm: Base = [%lf]", msg.base)
            rospy.loginfo("Arm: Shoulder = [%lf]", msg.shoulder)
            rospy.loginfo("Arm: Elbow = [%lf]", msg.elbow)
            rospy.loginfo("Arm: Fingers = [%lf]", msg.fingers)
            rospy.loginfo("Arm: Wrist = [%lf]", msg.wrist)
            arm_publisher.publish(msg)

    def check_origin(self, origin):
        return True

def ros_init():
    global arm_publisher
    global drive_publisher
    rospy.init_node('controller_proxy', log_level=rospy.INFO)
    rospy.loginfo("Initializing input node")
    rate = rospy.Rate(2)
    if rospy.is_shutdown():
        tornado.ioloop.IOLoop.current().stop()
    # Init publishers
    arm_publisher = rospy.Publisher('/user_arm_commands', input_arm, queue_size=10)
    drive_publisher = rospy.Publisher('/user_drive_commands', input_drive, queue_size=10)

if __name__ == "__main__":
    ros_init()
    application = tornado.web.Application([
        ("/websocket", ControllerHandler)
    ])
    application.listen(9090)
    try:
        tornado.ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        tornado.ioloop.IOLoop.current().stop()

    # while not rospy.is_shutdown():
    #     rate.sleep()
