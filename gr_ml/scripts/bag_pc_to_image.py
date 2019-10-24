#!/usr/bin/python
import rosbag
import argparse
from gr_ml.gr_terrain_recognition.pc_to_image import Lidar2Image


help_text = 'This is a script that converts PointCloud2 message to RGB images'

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--bag", "-b", help="set input bagname")
    parser.add_argument("--group", "-g", default="image")
    parser.add_argument("--topic", "-t", default="/velodyne_points")
    parser.add_argument("--meters", "-m", default=10)
    parser.add_argument("--pix_meters", "-p", default=15)
    parser.add_argument("--z_range", "-z", default=2.5)
    parser.add_argument("--debug", "-d", default=1)

    args = parser.parse_args()
    debug_mode = bool(int(args.debug))
    bag = rosbag.Bag(args.bag, mode="r")
    lidar2image = Lidar2Image(save_image=True, filegroup=args.group, meters = args.meters, pix_per_meter=args.pix_meters, z_range=args.z_range)
    lidar2image.save_params()

    for topic, msg, t in bag.read_messages(topics=args.topic):
        lidar2image.topic_cb(msg)
        if debug_mode:
            break
    bag.close()