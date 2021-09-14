from ..task import Task
from ..CommandProcessor.ComInterface.communicationHandler import CommunicationHandler
from ..CommandProcessor.ComInterface.mqttHandler import MqttHandler
import rospy
import json

class StatusMonitor:
    def __init__(self, rciComHandler, i2rComHandler):
        """ Status Monitor Class (Relay I2R Status to Robot Display)

            :rciComHandler   Robot Computer Interface Communication Handler
            :i2rComHandler   MQTT Handler to communication with I2R server
        """
        self.i2rComHandler = i2rComHandler
        self.rciComHandler = rciComHandler

        self.i2r_robot_status_topic = rospy.get_param(rospy.get_name() + "/i2r_robot_status_topic", "robot_status")
        self.i2r_robot_status_field = rospy.get_param(rospy.get_name() + "/i2r_robot_status_field", "status")

        self.i2rComHandler.subscribe(self.i2r_robot_status_topic, self.status_callback)

    def __call__(self, *args):
        pass

    def status_callback(self, msg):
        try:
            msgDict = json.loads(msg)
            self.rciComHandler.send_message(msgDict)
        except Exception as ex:
            rospy.logerr("%s: StatusMonitor: Cannot decode incoming message [%s]: %s"%(rospy.get_name(), msg,str(ex)))


if __name__ == "__main__":
    rospy.init_node("i2r_task")
    rospy.loginfo("I2R Task Unit Test")

    i2r_mqtt_broker_address = "localhost"
    i2r_mqtt_interface = MqttHandler("i2r_ros_mqtt_client", i2r_mqtt_broker_address)
    # task = StatusMonitor(comInterface=i2r_mqtt_interface)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()