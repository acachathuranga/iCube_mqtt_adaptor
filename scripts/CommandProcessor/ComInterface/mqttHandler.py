import paho.mqtt.client as mqtt
import rospy

class MqttHandler():
    def __init__(self, name, broker='localhost', port=1883, log=False):
        """ Mqtt Client

            :name       : Unique Client Name
            :broker     : Mqtt broker host address
            :port       : Mqtt broker port

            Note:
                The returned message string will be decoded with "utf-8".
                Hence make sure your mqtt messages are utf-8 decodable
        """

        self.name = name
        self.broker = broker
        self.port = port
        self.log = log

        self.client = mqtt.Client(name)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.msg_callback

        self.client.connect(broker,port)
        self.client.loop_start()

        self.callbacks = {}

    def __del__(self):
        """ Destructor
        """
        self.client.disconnect()
        self.client.loop_stop()

    def subscribe(self, topic, callback):
        self.callbacks[topic] = callback

    def publish(self, topic, msg):
        self.client.publish(topic, msg)

    def on_connect(self, client, userdata, flags, rc):
        """ Client Connected Callback
        """
        rospy.loginfo("%s: Client %s connected with result code %s"%(rospy.get_name(), self.name, str(rc)))
        # Subscribe to all topics
        self.client.subscribe("#")

    def msg_callback(self, client, userdata, msg):
        """ Message Callback Function
        """
        # Process message only if subscribed
        if msg.topic in self.callbacks:
            message = msg.payload.decode("utf-8")
            # Print messages if 'log' is enabled
            if self.log:
                rospy.loginfo("%s: [Topic %s]: %s"%(rospy.get_name(), msg.topic, message))
            # Message Callback
            if not(self.callbacks[msg.topic] is None):
                self.callbacks[msg.topic](message)


def printTest(msg):
    rospy.loginfo("onTest: " + msg)

def printStatus(msg):
    rospy.loginfo("onStatus: " + msg)

if __name__ == "__main__":
    rospy.init_node("mqtt_handler")
    rospy.loginfo("MQTT Handler Unit Test")
    mqtt_client = MqttHandler(name="testClient")

    mqtt_client.subscribe("test", printTest)
    mqtt_client.subscribe("status", printStatus)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        mqtt_client.publish("testPublish", "Hello World!")
        rate.sleep()

