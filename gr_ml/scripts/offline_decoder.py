#!/usr/bin/python
import argparse
from gr_ml.gr_terrain_recognition.decoder import ImageToPc

help_text = 'This is a script that converts RGB images to PointCloud2 messages'

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--folder", "-b", help="set input bagname", default=False)
    parser.add_argument("--extension", "-e", help="set image extension", default=".jpeg")
    parser.add_argument("--topic", "-t", default="/velodyne_points")
    parser.add_argument("--index", "-i", default=0)
    parser.add_argument("--ros", "-r", default=0)
    parser.add_argument("--ground_truth", "-gt", default=0)
    parser.add_argument("--debug", "-db", default=0)

    args = parser.parse_args()
    image2PC = ImageToPc(extension=args.extension, folder=args.folder, topic = args.topic,
                         index = args.index, enable_ros = args.ros,
                         ground_truth = args.ground_truth, debug_mode = args.debug)

    #image2PC.save_params()
    while True:
        current_image = image2PC.get_next_image()
        if current_image is None:
            break

        image2PC.compute_pc(current_image)

        if image2PC.task_done:
            break

    print("Finished")
