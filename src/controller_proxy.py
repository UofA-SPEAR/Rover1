#!/usr/bin/env python

import rospy
import json
import tornado.websocket
import tornado.ioloop
import tornado.web

from rover1.msg import input_arm
from rover1.msg import input_drive

class ControllerHandler(tornado.websocket.WebSocketHandler):
    def initialize(self):
        if rospy.is_shutdown():
            tornado.ioloop.IOLoop.current().stop()
        # Init publishers
        self.arm_publisher = rospy.Publisher('/user_arm_commands', input_arm, queue_size=10)
        self.drive_publisher = rospy.Publisher('/user_drive_commands', input_drive, queue_size=10)

    def on_message(self, data):
        json_msg = json.loads(data)
        if json_msg["type"] == "drive":
            # do the thing
            msg = input_drive
            msg.left = json_msg["left"]
            msg.right = json_msg["right"]
            rospy.loginfo("Drive: left = [%lf]", msg.left)
            rospy.loginfo("Drive: right = [%lf]", msg.right)
            self.drive_publisher.publish(msg)
        elif msg["type"] == "arm":
            # do the thing
            msg = input_arm
            msg.elbow = json_msg["elbow"]
            msg.wrist = json_msg["wrist"]
            msg.fingers = json_msg["fingers"]
            rospy.loginfo("Arm: Elbow = [%lf]", msg.elbow)
            rospy.loginfo("Arm: Fingers = [%lf]", msg.fingers)
            rospy.loginfo("Arm: Wrist = [%lf]", msg.wrist)
            self.arm_publisher.publish(msg)

    def check_origin(self, origin):
        return True

if __name__ == "__main__":
    rospy.init_node('controller_proxy', log_level=rospy.INFO)
    rate = rospy.Rate(2)
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
