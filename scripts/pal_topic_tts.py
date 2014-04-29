#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_topic_tts
#
# Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
#
# Authors:
#   * Siegfried-A. Gevatter

import argparse

import roslib; roslib.load_manifest('pal_debug_tts')
import rospy
import rostopic

from pal_python import pal_hci

class TopicMonitor:

    def __init__(self, topic, attr):
        msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic, blocking=True)
        rospy.Subscriber(real_topic, msg_class, self._callback)
        self._attr = attr
        self._triggers = {}
        self._actions = []
        self._last_value = None

    def add_action(self, action):
        assert callable(action)
        self._actions.append(action)

    def add_trigger(self, value, user_data):
        self._triggers[value] = user_data

    def run(self):
        rospy.spin()

    def _get_attr(self, value, attr):
        parts = attr.split('.')
        result = value
        for part in parts:
            result = getattr(value, part)
        return result

    def _callback(self, value):
        value = self._get_attr(value, self._attr)
        value = str(value)  # so it can be compared to the user-given values
        if value == self._last_value:
            return
        if value in self._triggers:
            user_data = self._triggers[value]
            for action in self._actions:
                action(user_data)
        self._last_value = value

def speak_message(message):
    print message
    pal_hci.tts_speak(message)

def main():
    rospy.init_node('pal_topic_tts', anonymous=True)

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description='Speak out when the value of a topic changes.\n\n'
                    'Example:\n\trosrun pal_debug_tts pal_topic_tts.py \\\n'
                    '\t\t--topic /emergency_lock --attr data \\\n'
                    '\t\t--value True "Emergency detected" \\\n'
                    '\t\t--value False "Okay"')
    parser.add_argument('--topic', dest='topic',
        help='Topic to watch.')
    parser.add_argument('--attr', dest='attr',
        help='Topic attribute to check.')
    parser.add_argument('--value', metavar=('VALUE', 'MESSAGE'), dest='values',
        nargs=2, action='append', required=True,
        help='Value to watch for and message to speak aloud.')
    args = parser.parse_args()

    m = TopicMonitor(args.topic, args.attr)
    m.add_action(speak_message)

    for entry in args.values:
        m.add_trigger(entry[0], entry[1])

    m.run()

if __name__ == '__main__':
    main()
