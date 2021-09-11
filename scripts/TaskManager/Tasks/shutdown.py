from ..task import Task
from ..CommandProcessor.ComInterface.mqttHandler import MqttHandler
import rospy

class Shutdown():
    def __init__(self, comHandler):
        """ Shutdown Robot Computer

            :comHandler   MQTT Handler to communication with I2R server
        """
        self.comHandler = comHandler

        self.i2r_command_topic = MqttHandler("i2r_command_topic", "robot_depart")
        self.i2r_command_field = MqttHandler("i2r_command_field", "command")

    def execute(self, **kwargs):
        msg = {self.i2r_command_field: "shutdown"}
        self.comHandler.publish(self.i2r_command_topic, msg)

if __name__ == "__main__":
    rospy.init_node("i2r_task")
    rospy.loginfo("I2R Task Unit Test")

    i2r_mqtt_broker_address = "localhost"
    i2r_mqtt_interface = MqttHandler("i2r_ros_mqtt_client", i2r_mqtt_broker_address)
    task = Shutdown(comInterface=i2r_mqtt_interface)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()