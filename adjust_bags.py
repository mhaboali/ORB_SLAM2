import rosbag

with rosbag.Bag('corrected_01.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('/home/h/vslam_ws/dataset/01_color.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if ((topic == "/image_left" or topic == "/image_right") and msg.data):
            outbag.write(topic, msg, msg.header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)