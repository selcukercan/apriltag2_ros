import rosbag
import rospy
from shutil import copy

input_base = "/home/selcuk/save_test9"
input_backup_bag = input_base + "_backup.bag"
input_bag = input_base + ".bag"
output_bag = input_base + "_synced.bag"

copy(input_bag, input_backup_bag)

t_start_compressed = None
top_compressed_image = "/mete/camera_node/image/compressed"
top_tag_detections = "/mete/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame"

topics2save = []
for topic, msg, t in rosbag.Bag(input_bag).read_messages():
    # This also replaces tf timestamps under the assumption
    # that all transforms in the message share the same timestamp
    if topic == top_compressed_image:
        #t_start_compressed = msg.header.stamp
        t_start_compressed = t
        break

get_dt = False
with rosbag.Bag(output_bag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        if topic == top_tag_detections:
            print "caught"

        if topic == top_tag_detections:
            if not get_dt:
                d = t - t_start_compressed
                dt = rospy.Time(d.secs, d.nsecs)
                get_dt = True

            norm_d = t - dt
            target_t = rospy.Time(norm_d.secs, norm_d.nsecs)
            outbag.write(topic, msg, target_t)

        else:
            outbag.write(topic, msg, t)

print 'finished'