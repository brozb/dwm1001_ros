#!/usr/bin/env python
import rospy
import tf2_ros
import sys

from geometry_msgs.msg import TransformStamped, Quaternion
from dwm1001_ros.msg import UWBMeas, TagLocation


class UWB_Pose:
    def __init__(self, world_frame, tag_id):
        rospy.init_node("uwb_pose_publisher")

        self.world = world_frame
        self.tag_id = tag_id
        self.added = []

        self.broadcaster1 = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster2 = tf2_ros.TransformBroadcaster()

        self.subs1 = rospy.Subscriber("distances", UWBMeas, self.add_anchors, queue_size=1)
        self.subs2 = rospy.Subscriber("pos_estimate", TagLocation, self.publish_estimate, queue_size=1)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.subs1.unregister()
        self.subs2.unregister()
        rospy.sleep(0.2)

    def add_anchors(self, msg):
        for an in msg.measurements:
            if an.id not in self.added:
                rospy.loginfo("Adding new anchor with ID %s" % an.id)
                self.added += [an.id]
                tf = TransformStamped()
                tf.header.frame_id = self.world
                tf.header.stamp = rospy.Time.now()
                tf.child_frame_id = "anchor_" + an.id
                tf.transform.translation.x = an.location.x
                tf.transform.translation.y = an.location.y
                tf.transform.translation.z = an.location.z
                tf.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)

                self.broadcaster1.sendTransform(tf)

    def publish_estimate(self, msg):
        tf = TransformStamped()
        tf.header.frame_id = self.world
        tf.header.stamp = rospy.Time.now()
        tf.child_frame_id = "tag_" + self.tag_id
        tf.transform.translation.x = msg.location.x
        tf.transform.translation.y = msg.location.y
        tf.transform.translation.z = msg.location.z
        tf.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)

        self.broadcaster2.sendTransform(tf)


if __name__ == "__main__":
    arg = rospy.myargv(argv=sys.argv)
    if len(arg) != 3:
        print("ERROR: wrong number of arguments")
        print("expected two: world_frame, tag_id")
        print("got:", arg)
        quit()
    u = UWB_Pose(arg[1], arg[2])
    rospy.spin()
