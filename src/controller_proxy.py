import rospy
import urrlib
import json
import tornado.websocket
import tornado.ioloop
import tornado.web

from rover1.msg import input_arm
from rover1.msg import input_drive

class ControllerHandler(tornado.websocket.WebSocketHandler):
    def __init__(self):
    # Init publishers
    arm_publisher = rospy.Publisher('/user_arm_commands', input_arm)
    drive_publisher = rospy.Publisher('/user_drive_commands', input_drive)

    def on_message(self, data):
        json_msg = json.load(data)
        if json_msg["type"] == "drive":
            # do the thing
            msg = input_drive
            msg.netAngle = json_msg["netAngle"]
            msg.netSpeed = json_msg["netSpeed"]
            drive_publisher.publish(msg)
        elif msg["type"] == "arm":
            # do the thing
            msg = input_arm
            msg.elbow = json_msg["elbow"]
            msg.wrist = json_msg["wrist"]
            msg.fingers = json_msg["fingers"]
            arm_publisher.publish(msg)


if __name == "__main__":
    application = tornado.web.Application([
        ("/websocket", ControllerHandler)
    ])
    application.listen(9090)
    tornado.IOLoop.current().start()

    rate = rospy.Rate(2)
    rospy.init_node('controller_proxy', log_level=rospy.INFO)
    while not rospy.is_shutdown():
        rate.sleep()
