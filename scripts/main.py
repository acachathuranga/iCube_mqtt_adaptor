#! /usr/bin/env python
import rospy
from TaskManager.Tasks.heartbeat import HeartBeat
from TaskManager.Tasks.shutdown import Shutdown
from TaskManager.Tasks.gripper_release import GripperRelease
from TaskManager.CommandProcessor.commandProcessor import CommandProcessor
from TaskManager.CommandProcessor.ComInterface.communicationHandler import CommunicationHandler
from TaskManager.CommandProcessor.ComInterface.mqttHandler import MqttHandler

class RobotAdaptor():
    def __init__():
        # Fetch parameters
        mqtt_broker_address = rospy.get_param(rospy.get_name() + "/mqtt_broker_address", "localhost")
        i2r_mqtt_broker_address = rospy.get_param(rospy.get_name() + "/i2r_mqtt_broker_address", "192.168.5.128")

        comHandler = CommunicationHandler(broker=mqtt_broker_address)
        commandProcessor = CommandProcessor(comHandler)

        i2r_mqtt_interface = MqttHandler("i2r_ros_mqtt_client", i2r_mqtt_broker_address)

        # Create Tasks
        heartbeat = HeartBeat(comHandler)
        shutdown = Shutdown(i2r_mqtt_interface)
        gripper_release = GripperRelease(i2r_mqtt_interface)
        


        # Register Tasks
        commandProcessor.register_task("heartbeat", heartbeat)



if __name__ == '__main__':
    rospy.init_node('robot_mqtt_adaptor')
    rospy.loginfo("Robot MQTT Adaptor Running")
    try:
        robotAdaptor = RobotAdaptor()   
    except Exception as ex:
        rospy.logwarn(rospy.get_name() + " : " + str(ex))
    rospy.spin()

