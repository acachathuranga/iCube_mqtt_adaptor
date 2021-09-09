#! /usr/bin/env python
import rospy

if __name__ == '__main__':
    rospy.init_node('robot_mqtt_adaptor')
    rospy.loginfo("Robot MQTT Adaptor Running")
    try:
        // TODO
    except Exception as ex:
        rospy.logwarn(rospy.get_name() + " : " + str(ex))
    rospy.spin()

