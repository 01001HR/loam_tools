#!/usr/bin/env python

#
# This code was lifted and hacked. The original is from:
#
# https://github.com/OSUrobotics/bag2video
#

from __future__ import division
import rosbag, rospy, numpy as np
import sys, os, cv2, glob
from itertools import izip, repeat
from subprocess import call
import argparse

# try to find cv_bridge:

frame_count = 0

try:
    from cv_bridge import CvBridge
except ImportError:
    # assume we are on an older ROS version, and try loading the dummy manifest
    # to see if that fixes the import error
    try:
        import roslib; roslib.load_manifest("bag2frames")
        from cv_bridge import CvBridge
    except:
        print "Could not find ROS package: cv_bridge"
        sys.exit(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract and encode video from bag files.')
    parser.add_argument('--outfile', '-o', action='store', default="frame_",
                        help='Base name of the extracted frame files. Defaults frame_.')
    parser.add_argument('--start', '-s', action='store', default=rospy.Time(0), type=rospy.Time,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=rospy.Time(sys.maxint), type=rospy.Time,
                        help='Rostime representing where to stop in the bag.')
    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8', '32FC1'), default='bgr8',
                        help='Encoding of the deserialized image.')

    parser.add_argument('topic')
    parser.add_argument('bagfile')

    args = parser.parse_args()


    for bagfile in glob.glob(args.bagfile):
        print bagfile
        bag = rosbag.Bag(bagfile, 'r')
        iterator = bag.read_messages(topics=args.topic, start_time=args.start, end_time=args.end)
        bridge = CvBridge()
        frame_count = 0
        win = cv2.namedWindow('win')
        for (topic, msg, time) in iterator:
            tstamp = msg.header.stamp.to_sec();
            img = np.asarray(bridge.imgmsg_to_cv2(msg, args.encoding))
            if args.encoding == 'bgr8':
                fname = '%s%06d.jpg' % (args.outfile, frame_count)
                cv2.imwrite(fname, img)
            else:
                img = np.nan_to_num(img);
                rg = [np.amin(img), np.amax(img)];
                print "range: %f - %f" % (rg[0], rg[1])
                img = (img - rg[0]) * 255/(rg[1]-rg[0]);
                fname = '%s%06d.bmp' % (args.outfile, frame_count)
                cv2.imwrite(fname, img)
            #cv2.imshow(win, img)
            #cv2.waitKey(1)
            print "wrote %s" % (fname)
            frame_count = frame_count + 1
    
