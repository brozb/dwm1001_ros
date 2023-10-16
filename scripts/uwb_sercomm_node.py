#!/usr/bin/python3
import serial
import time
import rospy
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import PointStamped

Nan = "NAN"
usb = "/dev/ttyACM0"


def serial_setup(usb_port):
    ser = serial.Serial(usb_port, 115200, timeout=0.1)
    if ser.is_open:
        rospy.loginfo("Serial comm started at :")
        return ser
    else:
        rospy.loginfo("Serial comm error at :")
        return 0


def dist_nan(msg_dist):
    msg_dist.quaternion.x = float("NAN")
    msg_dist.quaternion.y = float("NAN")
    msg_dist.quaternion.z = float("NAN")
    msg_dist.quaternion.w = float("NAN")
    return msg_dist


def pose_nan(msg_pose):
    msg_pose.point.x = float("NAN")
    msg_pose.point.y = float("NAN")
    msg_pose.point.z = float("NAN")
    return msg_pose


def pose_old(msg_pose):
    msg_pose.point.x = float(-1)
    msg_pose.point.y = float(-1)
    msg_pose.point.z = float(-1)
    return msg_pose


def dist_old(msg_dist):
    msg_dist.quaternion.x = float(-1)
    msg_dist.quaternion.y = float(-1)
    msg_dist.quaternion.z = float(-1)
    msg_dist.quaternion.w = float(-1)
    return msg_dist


def set_uwb_mode(ser):
    rospy.loginfo("Setting UWB tag")
    ser.write(b"\n")
    time.sleep(0.5)
    ser.write(b"\n")
    time.sleep(0.5)
    ser.write(b"les\n")
    time.sleep(0.5)
    ser.write(b"av\n")


def read_serial(ser):
    if ser.is_open:
        raw_data = ser.readline()
        data = raw_data.split()
        return data
    else:
        return Nan


def data_parce(data, msg_pose, msg_dist):
    count = 0
    dist_old(msg_dist)
    pose_old(msg_pose)
    for i in range(0, len(data)):
        rospy.loginfo(data[i])
    if count > 30:
        for i in range(0, len(data)):
            if data[i].find(b"9AA5") != -1:
                dist = data[i].split(b"=")
                msg_dist.quaternion.x = float(dist[1])

            if data[i].find(b"56B6") != -1:
                dist = data[i].split(b"=")
                msg_dist.quaternion.w = float(dist[1])

            if data[i].find(b"C102") != -1:
                dist = data[i].split(b"=")
                msg_dist.quaternion.y = float(dist[1])

            if data[i].find(b"8083") != -1:
                dist = data[i].split(b"=")
                msg_dist.quaternion.z = float(dist[1])

            if data[i].find(b"est") != -1:
                pose_raw = data[i].split(b"[")
                pose = pose_raw[1].split(b"]")
                pose_final = pose[0].split(b",")
                msg_pose.point.x = float(pose_final[0])
                msg_pose.point.y = float(pose_final[1])
                msg_pose.point.z = float(pose_final[2])
    return msg_dist, msg_pose


def uwb():
    rospy.init_node("uwb_localization")
    pub_dist = rospy.Publisher("uwb_dist", QuaternionStamped, queue_size=10)
    rospy.loginfo("publishing to the topic DIST")
    pub_pose = rospy.Publisher("uwb_pose", PointStamped, queue_size=10)
    rospy.loginfo("publishing to the topic POSE")

    ser = serial_setup(usb)
    if ser == 0:
        rospy.loginfo("Restart node, cannot open serial comm with UWB tag!!!")

    msg_pose = PointStamped()
    msg_dist = QuaternionStamped()
    dist_nan(msg_dist)
    pose_nan(msg_pose)
    set_uwb_mode(ser)
    while not rospy.is_shutdown():
        data = read_serial(ser)
        if data == Nan:
            rospy.loginfo("Ser comm was compromised")
            dist_nan(msg_dist)
            pose_nan(msg_pose)
            ser = serial_setup(usb)
            if ser == 0:
                rospy.loginfo("Restart node, cannot open serial comm with UWB tag!!!")
        msg = data_parce(data, msg_pose, msg_dist)
        msg_dist = msg[0]
        msg_pose = msg[1]

        time_now = rospy.Time.now()
        msg_dist.header.stamp = time_now
        msg_pose.header.stamp = time_now
        pub_dist.publish(msg_dist)
        pub_pose.publish(msg_pose)


if __name__ == "__main__":
    uwb()
