#!/usr/bin/python
import argparse
import rosbag
from gr_ml.gr_terrain_recognition.analyze_and_score import DetectionsScore


help_text = 'This is a script that converts PointCloud2 message to RGB images'

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--bag", "-b", help="set input bagname")
    parser.add_argument("--gt_group", "-gg", default="nibio_2019")
    parser.add_argument("--measured_group", "-mg", default="nibio_2019")
    parser.add_argument("--topic", "-t", default="/velodyne_points")
    parser.add_argument("--index", "-i", default=1)

    args = parser.parse_args()
    #debug_mode = bool(int(args.debug))

    bag = rosbag.Bag(args.bag, mode="r")
    scoring_utils = DetectionsScore(gt_location=args.gt_group, measured_location=args.measured_group, index=int(args.index))

    for topic, msg, t in bag.read_messages(topics=args.topic):
        scoring_utils.score(msg)
        if True:#debug_mode:
            break
    bag.close()
