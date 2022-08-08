"""
Capture interface
"""

import argparse
import time
import zmq
import os
import pyrealsense2 as rs
import numpy as np
import cv2


def record_data(args):
    """
    """

    print("Starting Recording...")


    if not args.pupil_labs_off:

        # pupil set up
        ctx = zmq.Context()
        pupil_remote = zmq.Socket(ctx, zmq.REQ)
        pupil_remote.connect(f"tcp://{args.pupil_labs_ip}:{args.pupil_labs_port}")

        # start pupil recording
        pupil_remote.send_string('R')
        print(f"Start Pupil recording: {pupil_remote.recv_string()}")


    if not args.realsense_off:

        # TODO: these should be input args
        color_path = '../data/realsense_rgb.avi'
        depth_path = '../data/realsense_depth.avi'

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
        pipeline.start(config)

        try:
            while True:
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

            # colorwriter.release()
            depthwriter.release()
            pipeline.stop()

        print("Ending depth recording")

    time.sleep(5)

    if not args.pupil_labs_off:

        pupil_remote.send_string('r')
        print(f"Ending Pupil recording")


if __name__ == '__main__':


    parser = argparse.ArgumentParser(
        prog='python capture.py',
         description="Capture data from Pupil tracker and Intel Realsense camera")

    # OPTIONS
    parser.add_argument("--output",
                        default="output.json",
                        help="path to output file. (default: output.json)")
    parser.add_argument("--max-frames-per-second",
                        default=70,
                        type=int,
                        help="sets the max number of frames captured per second. (default: 70)")

    # DEPTH SENSOR OPTIONS
    parser.add_argument('--realsense-off',
                        action='store_true',
                        help="don't record any data from RealSense.")
    # parser.add_argument("--realsense-ip",
    #                     default="127.0.0.1",
    #                     help="ip address for RealSense. (default: 127.0.0.1)")
    # parser.add_argument("--realsense-command-port",
    #                     default=1510,
    #                     type=int,
    #                     help="command port for RealSense. (default: 1510)")
    # parser.add_argument("--realsense-data-port",
    #                     default=1511,
    #                     type=int,
    #                     help="data port for RealSense. (default: 1511)")


    # PUPIL CAMERA OPTIONS
    parser.add_argument("--pupil-labs-ip",
                        default="127.0.0.1",
                        help="ip address for Pupil Labs. (default: 127.0.0.1)")
    parser.add_argument("--pupil-labs-port",
                        default=50020,
                        type=int,
                        help="port for Pupil Labs. (default: 50020)")
    parser.add_argument('--pupil-labs-off',
                        action='store_true',
                        help="don't record any data from pupil labs.")
    parser.add_argument('--pupil0-off',
                        action='store_true',
                        help="don't record any pupil.0 data from pupil labs.")
    parser.add_argument('--pupil1-off',
                        action='store_true',
                        help="don't record any pupil.1 data from pupil labs.")



    args = parser.parse_args()

    record_data(args)
