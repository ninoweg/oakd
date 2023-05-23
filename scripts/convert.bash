#! /bin/bash

ffmpeg -y -framerate 30 -i  ./records/right.h265 -c copy ./records/right.mp4
ffmpeg -y -framerate 30 -i ./records/left.h265 -c copy ./records/left.mp4
ffmpeg -y -framerate 30 -i ./records/rgb.h265 -c copy ./records/rgb.mp4
ffmpeg -y -framerate 30 -i ./records/preview.h265 -c copy ./records/preview.mp4
