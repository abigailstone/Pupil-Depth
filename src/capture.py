"""
Capture interface
"""

import argparse
import time
from datetime import datetime

import zmq
import os
import pyrealsense2 as rs
import numpy as np
import cv2


def record_data(args, output_path):
    """
    Record from camera(s) specified in command line args
    """

    print("Starting Recording...")


    if not args.pupil_off:

        # pupil set up
        ctx = zmq.Context()
        pupil_remote = zmq.Socket(ctx, zmq.REQ)
        pupil_remote.connect(f"tcp://{args.pupil_ip}:{args.pupil_port}")

        # start pupil recording
        pupil_remote.send_string('R')
        print(f"Start Pupil recording: {pupil_remote.recv_string()}")


    if not args.realsense_off:

        color_path = os.path.join(output_path, 'rs_rgb.avi')
        depth_path = os.path.join(output_path, 'rs_depth.avi')

        # realsense set up
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # video writers
        colorwriter = cv2.VideoWriter(
            filename=color_path,
            fourcc=cv2.VideoWriter_fourcc(*'XVID'),
            fps=30,
            frameSize=(640, 480),
            isColor=1
        )

        depthwriter = cv2.VideoWriter(
            filename=depth_path,
            fourcc=cv2.VideoWriter_fourcc(*'XVID'),
            fps=30,
            frameSize=(640, 480),
            isColor=1
        )

        # start recording
        print("Starting Depth Recording")
        start = time.time()
        pipeline.start(config)

        try:
            while time.time() - start < int(args.rec_length):

                # get frames from RealSense
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                #convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                colorwriter.write(color_image)
                depthwriter.write(depth_colormap)

                cv2.imshow('Stream', depth_colormap)

                if cv2.waitKey(1) == ord("q"):
                    break

        finally:

            colorwriter.release()
            depthwriter.release()
            pipeline.stop()

        print("Ending depth recording")

    if args.realsense_off:
        # time the pupil recording if the realsense stuff is skipped
        time.sleep(int(args.rec_length))
        
    if not args.pupil_off:
        # stop the pupil recording 
        pupil_remote.send_string('r')
        print(f"Ending Pupil recording")

        # move the files 
        date = datetime.today().strftime('%Y_%m_%d')
        pupil_default_dir = os.path.join("recordings", date, "000")
        pupil_output = os.path.join(output_path, 'pupil')
        
        os.makedirs(pupil_output, exist_ok=True)
        os.rename(pupil_default_dir, pupil_output)


if __name__ == '__main__':


    parser = argparse.ArgumentParser(
        prog='python capture.py',
         description="Capture data from Pupil tracker and Intel Realsense camera")

    # OPTIONS
    parser.add_argument("--output",
                        default="~/Data/Pupil-Depth/",
                        help="path to output directory")
    parser.add_argument("--max-frames-per-second",
                        default=70,
                        type=int,
                        help="sets the max number of frames captured per second. (default: 70)")
    parser.add_argument("--rec-length",
                        default=30,
                        help="recording length")


    # DEPTH SENSOR OPTIONS
    parser.add_argument('--realsense-off',
                        action='store_true',
                        help="don't record any data from RealSense.")


    # PUPIL CAMERA OPTIONS
    parser.add_argument("--pupil-ip",
                        default="127.0.0.1",
                        help="ip address for Pupil Labs. (default: 127.0.0.1)")
    parser.add_argument("--pupil-port",
                        default=50020,
                        type=int,
                        help="port for Pupil Labs. (default: 50020)")
    parser.add_argument('--pupil-off',
                        action='store_true',
                        help="don't record any data from pupil labs.")
    parser.add_argument('--pupil0-off',
                        action='store_true',
                        help="don't record any pupil.0 data from pupil labs.")
    parser.add_argument('--pupil1-off',
                        action='store_true',
                        help="don't record any pupil.1 data from pupil labs.")


    args = parser.parse_args()

    participant = input("Enter Participant ID: ")
    dist = input("Enter distance (ft): ")

    output_path = os.path.join(args.output, participant, dist)
    print(f'output path: {output_path}')

    record_data(args, output_path)
