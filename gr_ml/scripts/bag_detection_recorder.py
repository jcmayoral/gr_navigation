#!/usr/bin/python
import argparse
import rosbag
from gr_ml.gr_data_acquisition.record_detections import DetectionRecorder

help_text="This script stores detection results"

if __name__ == '__main__':
    if __name__ == '__main__':
        parser = argparse.ArgumentParser(description = help_text)
        parser.add_argument("--bag", "-b", help="set input bagname")
        parser.add_argument("--group", "-g", default="test_2019")
        parser.add_argument("--topic", "-t", default="/velodyne_points")
        parser.add_argument("--debug", "-d", default=1)

        args = parser.parse_args()
        debug_mode = bool(int(args.debug))
        bag = rosbag.Bag(args.bag, mode="r")
        recoder = DetectionRecorder(folder=args.group)

        for topic, msg, t in bag.read_messages(topics=args.topic):
            recoder.publish_gt(msg)
            while not recoder.message_processed:
                pass
        bag.close()
