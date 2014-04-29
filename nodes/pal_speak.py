#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_speak
#
# Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
#
# Authors:
#   * Siegfried-A. Gevatter

import argparse

import roslib; roslib.load_manifest('pal_debug_tts')
import rospy

from pal_python import pal_hci

def main():
    parser = argparse.ArgumentParser(
        description='Speak a single sentence from the CLI.')
    parser.add_argument('sentence', help='Sentence to speak out aloud.')
    args = parser.parse_args()

    rospy.init_node('pal_speak', anonymous=True)
    pal_hci.tts_speak(args.sentence)

if __name__ == '__main__':
    main()
