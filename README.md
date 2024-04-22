# ROS driver for Qorvo DWM1001
Node `uwb_tag.py` communicates with the tag over UART and publishes range measurements as a custom message [UWBMeas](msg/UWBMeas.msg). If enough anchors are present,
the estimated position of the tag with respect to the anchors is published as a custom message [TagLocation](msg/TagLocation.msg).

# Usage
Two launch files are provided, the main launch file is [tag_node.launch](launch/tag_node.launch). This launches `uwb_tag.py` node for a single tag.
The provided ID of the tag is used to set a namespace for the node. The data for a tag with ID 5772 is therefore published on topics
`/uwb/5772/distances` and `/uwb/5772/pos_estimate`.

The second node is an example of using a single launch file for multiple tags.

The launch file provides the option (argument `publish_tfs`) to publish the estimated position of the tag with respect to the anchors as a tf2 frame. The position of the anchors is taken
from the messages received by the tag. For correct functioning of the frames publishing, it is necessary to set the fixed position of each tag through the app or UART interface.

The driver is capable of restoring the connection if it is temporarily lost (for example due to vibrations), it is recommended to set udev rule for each UWB module (each of them has a different serial number):

`ACTION=="add", ENV{ID_VENDOR_ID}=="1366", ENV{ID_MODEL_ID}="0105", ENV{ID_SERIAL_SHORT}=="000760166638", SYMLINK+="ttyACM_uwbc694"`
