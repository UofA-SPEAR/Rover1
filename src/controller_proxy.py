#!/usr/bin/env python

import rospy
import json
import tornado.websocket
import tornado.ioloop
import tornado.web
import threading
import time

from rover1.msg import input_arm
from rover1.msg import input_drive
from rover1.msg import output_sensors

clients = []

class ControllerHandler(tornado.websocket.WebSocketHandler):
    def on_close(self):
        print("Disconnecting")
        clients.remove(self)

    def open(self):
        print("Connecting")
        global clients
        clients.append(self)


    def on_message(self, data):
        global arm_publisher
        global drive_publisher
        json_msg = json.loads(data)
        #print(json_msg)
        if json_msg["type"] == "drive":
            # do the thing
            msg = input_drive()
            msg.left = json_msg["left"]
            msg.right = json_msg["right"]
            msg.wheelie = json_msg["wheelie"]
            #rospy.loginfo("Drive: left = [%lf]", msg.left)
            #rospy.loginfo("Drive: right = [%lf]", msg.right)
            drive_publisher.publish(msg)
        elif json_msg["type"] == "arm":
            # do the thing
            msg = input_arm()
            msg.base = json_msg["base"]
            msg.shoulder = json_msg["shoulder"]
            msg.elbow = json_msg["elbow"]
            msg.wrist_pitch = json_msg["wrist_pitch"]
            msg.wrist_roll = json_msg["wrist_roll"]
            msg.fingers = json_msg["fingers"]

            #rospy.loginfo("Arm: Base = [%lf]", msg.base)
            #rospy.loginfo("Arm: Shoulder = [%lf]", msg.shoulder)
            #rospy.loginfo("Arm: Elbow = [%lf]", msg.elbow)
            #rospy.loginfo("Arm: Wrist Pitch = [%lf]", msg.wrist_pitch)
            #rospy.loginfo("Arm: Wrist Roll = [%lf]", msg.wrist_roll)
            #rospy.loginfo("Arm: Fingers = [%lf]", msg.fingers)
            arm_publisher.publish(msg)

    def check_origin(self, origin):
        return True

def writeSensors(data):
    # TODO: send to base
    #rospy.loginfo(str(data))
    for c in clients:
        c.write_message(json.dumps(
            {"type": "science",
                "gps":{"lon":data.longtitude, "lat":data.latitude},
                "uv": data.uv,
                "temperature": data.temperature,
                "humidity": data.humidity,
                }
            ))
    

def ros_init():
    global arm_publisher
    global drive_publisher
    global sensor_subscriber

    rospy.init_node('controller_proxy', log_level=rospy.INFO)
    rospy.loginfo("Initializing input node")

    # Init publishers
    arm_publisher = rospy.Publisher('/user_arm_commands', input_arm, queue_size=50)
    drive_publisher = rospy.Publisher('/user_drive_commands', input_drive, queue_size=50)

    sensor_subscriber = rospy.Subscriber('/sensor_out', output_sensors, writeSensors)


    # do the thing
    msg = input_arm()
    msg.base = 0.5
    msg.shoulder =0.5
    msg.elbow = 0.5
    # we need to update the control panel to send messages for wrist pitch - Ryan
    msg.wrist_pitch = 0.5
    msg.wrist_roll = 0.5
    msg.fingers = 0.5
    arm_publisher.publish(msg)


class SpinThread(threading.Thread):
    def __init__(self):
        super(SpinThread, self).__init__()

    def run(self):
        rospy.spin()

class WebThread(threading.Thread):
    def __init__(self):
        super(WebThread, self).__init__()

    def run(self):
        application = tornado.web.Application([
            ("/websocket", ControllerHandler)
        ])
        application.listen(9090)
        tornado.ioloop.IOLoop.current().start()


t_ros = SpinThread()
t_web = WebThread()
if __name__ == "__main__":
    ros_init()

    t_ros.daemon = True
    t_ros.start()

    t_web.daemon = True
    t_web.start()

    while not rospy.is_shutdown():
        time.sleep(1) # keep everything alive until this loop exits, 
        # but also, busy looping is bad. So one second sleep


