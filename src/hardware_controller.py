#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import BotCmd
from hb_interfaces.msg import BotCmdArray
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
import sys


broker_ip = "10.45.228.66"
sensor_topic_0 = "esp/sensor/0"
sensor_topic_2 = "esp/sensor/2"
sensor_topic_4 = "esp/sensor/4"

detection_0 = False
detection_2 = False
detection_4 = False

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        client.subscribe(sensor_topic_0)
        client.subscribe(sensor_topic_2)
        client.subscribe(sensor_topic_4)

    else:
        print(f"Connection failed with code {rc}")
        sys.exit(1)

def on_message(client, userdata, msg):
    global detection_0 , detection_2 , detection_4
    detection_status = msg.payload.decode().strip()
    if(msg.topic == "esp/sensor/0"):
        if(detection_status == "True"):
            detection_0 = True
        else:
            detection_0 = False
    if(msg.topic == "esp/sensor/2"):
        if(detection_status == "True"):
            detection_2 = True
        else:
            detection_2 = False
    if(msg.topic == "esp/sensor/4"):
        if(detection_status == "True"):
            detection_4 = True
        else:
            detection_4 = False
        


def on_disconnect(client, userdata, rc):
    print("Disconnected from broker")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

class wheel_control(Node):
    def __init__(self):
        super().__init__('send_sppeds')
        self.motor_sub = self.create_subscription(BotCmdArray , '/bot_cmd' , self.cmd_callback , 10)
        self.attach_srv_0 = self.create_service(SetBool , '/attach/bot0', self.attach_callback_0)
        self.attach_srv_2 = self.create_service(SetBool , '/attach/bot2', self.attach_callback_2)
        self.attach_srv_4 = self.create_service(SetBool , '/attach/bot4', self.attach_callback_4)
        self.sensor_publisher_timer = self.create_timer(0.01 , self.sensor_publisher)
        self.sensor_0_pub = self.create_publisher(Bool , '/sensor0' , 10)
        self.sensor_2_pub = self.create_publisher(Bool , '/sensor2' , 10)
        self.sensor_4_pub = self.create_publisher(Bool , '/sensor4' , 10)


        client.connect(broker_ip)
        client.loop_start()

    def attach_callback_0(self , request , response):
        if request.data is True:
            client.publish("esp/link/0" , 'on')
        else:
            client.publish("esp/link/0", 'off')
            
        response.success = True
        response.message = "Link request sent...."
        return response
    
    def attach_callback_2(self , request , response):
        if request.data is True:
            client.publish("esp/link/2" , 'on')
        else:
            client.publish("esp/link/2", 'off')
            
        response.success = True
        response.message = "Link request sent...."
        return response
    
    def attach_callback_4(self , request , response):
        if request.data is True:
            client.publish("esp/link/4" , 'on')
        else:
            client.publish("esp/link/4", 'off')
            
        response.success = True
        response.message = "Link request sent...."
        return response

    def cmd_callback(self , Msg : BotCmdArray):
        #msg = Msg.cmds[0]
        for msg in Msg.cmds:
            cmd = str(int(msg.m1)) + "," + str(int(msg.m2)) + "," + str(int(msg.m3)) + "," + str(int(msg.base)) + "," + str(int(msg.elbow))
            if(msg.id == 0):
                client.publish("esp/cmd/0" , cmd)
            if(msg.id == 2):
                client.publish("esp/cmd/2" , cmd)
            if(msg.id == 4):
                client.publish("esp/cmd/4" , cmd)
    
    def sensor_publisher(self):
        msg0 = Bool()
        msg2 = Bool()
        msg4 = Bool()

        msg0.data = detection_0

        self.sensor_0_pub.publish(msg0)

        msg2.data = detection_2
        
        self.sensor_2_pub.publish(msg2)

        msg4.data = detection_4
        
        self.sensor_4_pub.publish(msg4)
        
            

def main(args=None):
    rclpy.init(args=args)
    pub = wheel_control()
    try:
        rclpy.spin(pub)
    except KeyboardInterrupt:
        pass
    finally:
        pub.destroy_node()
        client.loop_stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
