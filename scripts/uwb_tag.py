#!/usr/bin/python3
import serial
import rospy
import sys

from dwm1001c_ros.msg import UWBMeas, Anchor, TagLocation


class UWB_Tag:
    def __init__(self, usb_port: str, freq: float) -> None:
        rospy.init_node("uwb_tag")
        self.usb = usb_port

        self.ser = None
        self.tim = None
        self.received = 0
        self.failures = 0

        # open the port and start publishing
        self.serial_setup()
        rospy.sleep(0.5)
        if self.ser is not None:
            self.set_uwb_mode()

            self.pub1 = rospy.Publisher("distances", UWBMeas, queue_size=1)
            self.pub2 = rospy.Publisher("pos_estimate", TagLocation, queue_size=1)

            self.tim = rospy.Timer(rospy.Duration(1 / freq), self.read_data)
        else:
            rospy.logfatal("Serial port can not be opened, quitting")

        self.tim2 = rospy.Timer(rospy.Duration(3), self.test_connection)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.tim2.shutdown()
        rospy.sleep(0.2)
        if self.tim is not None:
            self.tim.shutdown()
            rospy.sleep(0.2)
        if self.ser is not None:
            self.close_serial()
            self.ser.close()
        rospy.loginfo("Shutting down")

    def test_connection(self, _):
        if self.received == 0:
            rospy.logwarn("No data received in the last 3 seconds")
            self.failures += 1
        else:
            self.received = 0
            self.failures = 0
        if self.failures == 3:
            rospy.logfatal("No data received in the last 9 seconds, quitting")
            rospy.signal_shutdown("no data")

    def serial_setup(self):
        try:
            self.ser = serial.Serial(self.usb, 115200, timeout=0.1)
            if self.ser.is_open:
                rospy.loginfo("Serial comm started at : %s" % self.usb)
            else:
                rospy.logfatal("Can't open %s" % self.usb)
                rospy.signal_shutdown("serial communication failed")
        except serial.SerialException:
            rospy.logfatal("Can't open %s" % self.usb)
            rospy.signal_shutdown("serial communication failed")

    def close_serial(self):
        rospy.loginfo("Closing connection")
        try:
            self.ser.write(b"les\r")
        except serial.SerialException as ex:
            rospy.logfatal("Serial exception [%s]" % ex)
        rospy.sleep(0.1)
        rospy.loginfo("Connection to tag closed")

    def set_uwb_mode(self):
        rospy.loginfo("Setting UWB tag")
        self.ser.write(b"\r")
        rospy.sleep(0.5)
        self.ser.write(b"\r")
        rospy.sleep(0.5)
        self.ser.write(b"les\r")
        rospy.sleep(0.5)
        rospy.loginfo("Setup done")

    def read_serial(self):
        if self.ser.is_open:
            try:
                raw_data = self.ser.readline()
            except serial.SerialException as ex:
                rospy.logfatal("Serial exception [%s]" % ex)
                rospy.signal_shutdown("Connection failed")
            data = raw_data.split()
            return data
        else:
            return None

    def read_data(self, _):
        data = self.read_serial()  # list of bytes
        if data is None:
            rospy.logerror("Communication compromised, trying to reset the port")
            self.ser = self.serial_setup()
        else:
            self.received += 1
            meas = UWBMeas()
            est = TagLocation()
            est_received = False
            no_warn = False
            for m in data:
                m_str = m.decode("utf-8")
                if len(m_str) >= 25 and m_str[4] == "[":
                    # distance from one of the anchors
                    # e.q. 1151[5.00,8.00,2.25]=6.48
                    a = Anchor()
                    a.id = m_str[0:4]  # 1151

                    p = m_str[5:19].split(",")  # list, ['5.00', '8.00', '2.25']
                    a.location.x = float(p[0])
                    a.location.y = float(p[1])
                    a.location.z = float(p[2])

                    a.dist = float(m_str[21:])  # 6.48
                    meas.measurements += [a]
                elif m_str[0:5] == "le_us":
                    # computation time
                    # e.g. le_us=2576
                    est.computation_time = int(m_str[6:10])  # 2576
                    est_received = True
                elif m_str[0:3] == "est":
                    # estimated position
                    # e.g. est[2.57,1.98,1.68,100]
                    d = m_str[4:23].split(",")  # list, ['2.57', '1.98', '1.68', '100']
                    est.location.x = float(d[0])
                    est.location.y = float(d[1])
                    est.location.z = float(d[2])
                    est.quality = int(d[3])
                elif m_str[0:3] == "dwm":
                    no_warn = True
                    break
                elif m_str in ["DWM1001", "Copyright", "License", "Compiled", "Help"]:
                    # welcome message
                    no_warn = True
                    if m_str == "DWM1001":
                        rospy.loginfo("Welcome message from the tag:")
                    s = ""
                    for i in range(len(data) - 1):
                        s += data[i].decode("utf-8") + " "
                    s += data[-1].decode("utf-8")
                    rospy.loginfo(s)
                    break
                else:
                    rospy.logerr("Unknown message from the tag [%s]" % (data))
            if len(meas.measurements) == 0 and not no_warn:
                rospy.logwarn("Message with no range data, received:[%s]" % str(data))
            self.pub1.publish(meas)
            if est_received:
                self.pub2.publish(est)


if __name__ == "__main__":
    arg = rospy.myargv(argv=sys.argv)
    if len(arg) != 3:
        print("ERROR: wrong number of arguments")
        print("expected two: usb_port, read_frequency")
        print("got:", arg[1:])
        quit()
    u = UWB_Tag(arg[1], float(arg[2]))
    rospy.spin()
