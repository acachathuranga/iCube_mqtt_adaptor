from ..task import Task
from ..CommandProcessor.ComInterface.communicationHandler import CommunicationHandler
import rospy
import threading

class HeartBeat:
    def __init__(self,  comInterface, interval=5):
        """ Sends a mqtt heartbeat at a defined interval

            :comInterface   CommunicationHandler
            :interval       Heartbeat sending interval

            Note: Heartbeat output from this class is an empty message
        """
        self.communicationHandler = comInterface
        self.rate = rospy.Rate(float(1.0/interval))

        # Start command processor thread
        thread = threading.Thread(target=self.sendHeartbeat)
        thread.start()

    def getState(self):
        if rospy.is_shutdown():
            return Task.TaskState.Error, "{\"heartbeat_status\":\"error\"}"    
        else:
            return Task.TaskState.Ok, "{\"heartbeat_status\":\"ok\"}"        

    def __call__(self, *args):
        empty_msg = {}
        self.communicationHandler.send_message(empty_msg)
    
    def sendHeartbeat(self):
        while not(rospy.is_shutdown()):
            empty_msg = {}
            self.communicationHandler.send_message(empty_msg)
            self.rate.sleep()



if __name__ == "__main__":
    rospy.init_node("heartbeat")
    rospy.loginfo("Heartbeat Task Unit Test")

    comInterface = CommunicationHandler()
    tassk = HeartBeat(comInterface=comInterface)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()