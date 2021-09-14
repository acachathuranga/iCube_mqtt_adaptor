from ..task import Task
from ..CommandProcessor.ComInterface.communicationHandler import CommunicationHandler
from ..CommandProcessor.ComInterface.mqttHandler import MqttHandler
import rospy
import json
import threading
from std_msgs.msg import String

class ObstacleDetection:
    obstacle_state = ""

    def __init__(self, comHandler):
        """ Obstacle Detection Task (Relay Obstacle Detection Status to Robot Display)

            :comHandler   Robot Computer Interface Communication Handler
        """
        self.comHandler = comHandler

        self.obstacle_status_topic = rospy.get_param(rospy.get_name() + "/obstacle_status_topic", "obstacle_detection")
        self.robot_obstacle_status_field = rospy.get_param(rospy.get_name() + "/robot_obstacle_status_field", "obstacle_detection")

        # Start display thread
        self.event = threading.Condition()
        thread = threading.Thread(target=self.task_thread)
        thread.start()
        rospy.Subscriber(self.obstacle_status_topic, String, self.obstacle_detection_callback)

    def __call__(self, *args):
        pass
    
    def obstacle_detection_callback(self, msg):
        if (self.obstacle_state != msg.data):
            self.obstacle_state = msg.data
            with self.event:
                self.event.notifyAll()

    def task_thread(self):
        # Conditional wait is used to throttle the obstacle state publish rate to robot display
        while not(rospy.is_shutdown()):
            with self.event:
                # Thread wait
                self.event.wait(5)
                self.comHandler.send_message({self.robot_obstacle_status_field: self.obstacle_state})

if __name__ == "__main__":
    rospy.init_node("i2r_task")
    rospy.loginfo("I2R Task Unit Test")

    comInterface = CommunicationHandler()
    tassk = ObstacleDetection(comInterface=comInterface)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()