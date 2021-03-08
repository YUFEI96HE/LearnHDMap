#!/bin/sh
cd
cd /usr/local/ffmpeg/bin
ffmpeg -input_format mjpeg -s 320x240 -i /dev/video0 -vcodec libx264 -preset:v ultrafast -tune:v zerolatency  -f flv rtmp://admin.techinao.com:9012/live/stream$1-0
