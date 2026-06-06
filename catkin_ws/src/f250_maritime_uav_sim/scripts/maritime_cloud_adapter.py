#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2


class CloudAdapter:
    def __init__(self):
        rospy.init_node("maritime_cloud_adapter")
        self.input_topic = rospy.get_param("~input_topic", "/maritime/deterministic_obstacles_cloud")
        self.output_topic = rospy.get_param("~output_topic", "/maritime/obstacles_cloud")
        self.frame_id = rospy.get_param("~frame_id", "")
        self.source_type = rospy.get_param("~source_type", "deterministic")
        self.publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber(self.input_topic, PointCloud2, self.cloud_callback, queue_size=1)
        rospy.loginfo("maritime cloud adapter source=%s input=%s output=%s",
                      self.source_type, self.input_topic, self.output_topic)

    def cloud_callback(self, msg):
        output = PointCloud2()
        output.header = msg.header
        if self.frame_id:
            output.header.frame_id = self.frame_id
        output.height = msg.height
        output.width = msg.width
        output.fields = msg.fields
        output.is_bigendian = msg.is_bigendian
        output.point_step = msg.point_step
        output.row_step = msg.row_step
        output.data = msg.data
        output.is_dense = msg.is_dense
        self.publisher.publish(output)


def main():
    CloudAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
