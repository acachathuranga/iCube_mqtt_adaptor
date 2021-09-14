#! /usr/bin/env python
import rospy
import traceback
from TaskManager.Tasks.heartbeat import HeartBeat
from TaskManager.Tasks.shutdown import Shutdown
from TaskManager.Tasks.initialize import Initialize
from TaskManager.Tasks.gripper_release import GripperRelease
from TaskManager.Tasks.status_monitor import StatusMonitor
from TaskManager.Tasks.obstacle_detection import ObstacleDetection
from TaskManager.CommandProcessor.commandProcessor import CommandProcessor
from TaskManager.CommandProcessor.ComInterface.communicationHandler import CommunicationHandler
from TaskManager.CommandProcessor.ComInterface.mqttHandler import MqttHandler

class RobotAdaptor():
    def __init__(self):
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
        dock_and_initialize = Initialize(i2r_mqtt_interface)
        status_monitor = StatusMonitor(comHandler, i2r_mqtt_interface)
        obstacle_detection = ObstacleDetection(comHandler)


        # Register Tasks
        commandProcessor.register_task("heartbeat", heartbeat)
        commandProcessor.register_task("shutdown", shutdown)
        commandProcessor.register_task("gripper_release", gripper_release)
        commandProcessor.register_task("dock", dock_and_initialize)
        commandProcessor.register_task("status_monitor", status_monitor)
        commandProcessor.register_task("obstacle_detect", obstacle_detection)



if __name__ == '__main__':
    rospy.init_node('robot_mqtt_adaptor')
    rospy.loginfo("Robot MQTT Adaptor Running")
    
    try:
        robotAdaptor = RobotAdaptor()   
    except Exception as ex:
        rospy.logwarn(rospy.get_name() + " : " + str(ex) + "  Traceback: " + traceback.format_exc())
    rospy.spin()

