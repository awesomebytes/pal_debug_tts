Utilities:

 * pal_speak:
       command-line tool that speaks out a single sentence.
       Example:
           ```
rosrun pal_debug_tts pal_speak.py "Hello world"
```
 * pal_debug_tts:
       "tail -f" like utility that reads messages written to a file and
       speaks them out. Multiple regexes can be provided to choose which
       messages should be spoken.
       Example (it may spam a lot!):
           ```
rosrun pal_debug_tts pal_debug_tts.py /var/log/syslog
```
 * pal_topic_tts:
       speaks out when the value of a topic changes.
       Example:
           ```
rosrun pal_debug_tts pal_topic_tts.py --topic /foo --attr data \
               --value True "Attribute .data of topic /foo changed to true"
               --value False "Attribute .data of topic /foo changed to false"
```

For more information, run the scripts with --help.
