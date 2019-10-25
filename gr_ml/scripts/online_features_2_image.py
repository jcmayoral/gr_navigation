#!/usr/bin/python
import argparse
from gr_ml.gr_terrain_recognition.coder import Features2Image


help_text = 'This is a script that converts FoundObjectsArray message to RGB images'

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--topic", "-t", default="/found_object")
    parser.add_argument("--meters", "-m", default=10)
    parser.add_argument("--pix_meters", "-p", default=1)
    parser.add_argument("--z_range", "-z", default=2.5)

    args = parser.parse_args()
    lidar2image = Features2Image(ros=True, save_image=False, meters = args.meters, pix_per_meter=args.pix_meters, z_range=args.z_range)
    lidar2image.save_params()