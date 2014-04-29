#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_debug_tts
#
# Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
#
# Authors:
#   * Siegfried-A. Gevatter

import argparse
import re

import roslib; roslib.load_manifest('pal_debug_tts')
import rospy

from pal_python import pal_hci

# This is a terrible idea and results in weird stuff happening on Ctrl-C.
# Catching SIGINT we can kind of do something, but it still sucks.
#def file_as_stream(filename):
#    stream = subprocess.Popen(["tail", "-n0", "-f", filename], stdout=subprocess.PIPE)
#    return os.fdopen(stream.stdout.fileno())

def file_stream(filename, skip=False):
    min_sleep = 0.0005
    sleep_incr = 0.0005
    max_sleep = 1.0

    f = open(filename)
    if skip:
        f.seek(0, 2)  # advance to end of file

    sleep = min_sleep
    while True:
        line = f.readline()
        if not line:
            rospy.sleep(sleep)
            if sleep < max_sleep:
                sleep += sleep_incr
            continue
        sleep = min_sleep
        yield line

class DebugMonitor:

    def __init__(self, input_filename):
        self._input_filename = input_filename
        self._filters = []
        self._actions = []

    def add_filter(self, func):
        assert callable(func)
        self._filters.append(func)

    def add_regex_filter(self, expression):
        regexp = re.compile(expression)
        def regex_filter(line):
            return regexp.match(line) is not None
        self.add_filter(regex_filter)

    def add_action(self, action):
        assert callable(action)
        self._actions.append(action)

    def run(self):
        strip_control_chars = dict.fromkeys(range(32))
        for line in file_stream(self._input_filename, skip=True):
            line = unicode(line).strip().translate(strip_control_chars)
            # TODO: Do this some better way...
            line = line.replace('[0m', '')
            if self._filters and not any(f(line) for f in self._filters):
                continue
            for action in self._actions:
                action(line)

def parse_ros_log(line):
    # This can be useful if we ever want to check the timestamps:
    #groups = re.findall(r'\[\s*(.*?)\s*\]', line)
    message = re.match(r'(?:\[.*?\]\s*)*:?\s*(.*)', line)
    if not message:
        return
    return message.group(1)

def speak_message(line):
    pal_hci.tts_speak(line)

def main():
    rospy.init_node('pal_debug_tts', anonymous=True)

    parser = argparse.ArgumentParser(
        description='Speak out text from a file stream.')
    parser.add_argument('filename', help='File to monitor for new lines.')
    parser.add_argument('--roslog', dest='roslog', action='store_true',
        help='Assume ROS log (removes all the [...] stuff). [default]')
    parser.add_argument('--no-roslog', dest='roslog', action='store_false',
        help='Disables ROS output parsing.')
    parser.add_argument('--regex', dest='regexes', action='append', default=[],
        help='Regex to match which lines will be printed. Multiple can be specified.')
    parser.set_defaults(roslog=True)
    args = parser.parse_args()

    m = DebugMonitor(args.filename)

    for regex in args.regexes:
        m.add_regex_filter(regex)

    if args.roslog:
        m.add_action(lambda line: speak_message(parse_ros_log(line)))
    else:
        m.add_action(speak_message)

    m.run()

if __name__ == '__main__':
    main()
