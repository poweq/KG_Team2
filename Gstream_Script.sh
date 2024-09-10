#!/bin/bash
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay config-interval=1 pt=96 ! gdppay ! tcpserversink host=0.0.0.0 port=5000

