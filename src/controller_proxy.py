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
from rover1.msg import input_rope
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
            #rospy.loginfo("Drive: left = [%lf]", msg.left)
            #rospy.loginfo("Drive: right = [%lf]", msg.right)
            drive_publisher.publish(msg)
        elif json_msg["type"] == "arm":
            # do the thing
            msg = input_arm()
            msg.pot = int(json_msg["shoulder"])
            arm_publisher.publish(msg)
        elif json_msg["type"] == "rope":
            msg = input_rope()
            rope.speed = json_msg["speed"]

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
    rope_publisher = rospy.Publisher('/rope_sub', input_rope, queue_size=50)

    sensor_subscriber = rospy.Subscriber('/sensor_out', output_sensors, writeSensors)




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


