#!/bin/sh
#/usr/local/ffmpeg/bin/ffmpeg -f video4linux2  -s 640x480 -i /dev/video1 -vcodec libx264 -preset:v ultrafast -tune:v zerolatency  -f flv rtmp://120.27.144.89:9012/live/stream1587089272051
#cd
#cd /usr/local/ffmpeg/bin
#./ffmpeg -f video4linux2  -s 640x480 -i /dev/video1 -vcodec libx264 -preset:v ultrafast -tune:v zerolatency  -f flv rtmp://admin.techinao.com:9012/live/stream432515341168414720
ffmpeg -input_format mjpeg -s 320x240 -i /dev/video1 -vcodec libx264 -preset:v ultrafast -tune:v zerolatency  -f flv rtmp://admin.techinao.com:9012/live/stream432515341168414720
