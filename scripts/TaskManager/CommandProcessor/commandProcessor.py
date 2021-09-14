import threading
import rospy
from ComInterface.communicationHandler import CommunicationHandler

class CommandProcessor():

    def __init__(self, communicationInterface):
        """ Command Processor Class 

            : comHandler : Communication Handler
        """
        self.tasks = {}
        self.comInterface = communicationInterface

        self.robot_command_field = rospy.get_param(rospy.get_name() + "/robot_command_field", "command")

        # Start command processor thread
        thread = threading.Thread(target=self.process_commands)
        thread.start()

    def register_task(self, command, task):
        """ Registers a task with the corresponding command

            command : String command 
            task    : object extending Task Class
        """
        self.tasks[command] = task

    def get_tasks(self):
        """ Returns all tasks registered in command processor
        """
        return self.tasks

    def process_commands(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if (self.comInterface.is_message_available()):
                msg = self.comInterface.get_message()

                if not(msg is None):
                    # Successfully decoded message received
                    if (self.robot_command_field in msg):
                        command = msg[self.robot_command_field]
                        if (command in self.tasks):
                            # Corresponding task available
                            rospy.loginfo("%s: CommandProcessor: Command Received [%s]"%(rospy.get_name(),msg))
                            self.tasks[command](msg)
                        else:
                            rospy.logwarn("%s: CommandProcessor: Unknown Command [%s]"%(rospy.get_name(),msg))
                    else:
                        rospy.logerr("%s: CommandProcessor: Invalid Command [%s]"%(rospy.get_name(), msg))
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("commandProcessor")
    rospy.loginfo("Command Processor Unit Test")

    comInterface = CommunicationHandler()
    commandProcessor = CommandProcessor(comInterface)

    msg = {"robot_status":"ok"}
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        commandProcessor.comInterface.send_message(msg)
        rate.sleep()