#! /bin/bash

ffmpeg -i ./records/rgb.h265   -filter:v fps=10 ./records/rgb.mp4
ffmpeg -i ./records/left.h265  -filter:v fps=10 ./records/left.mp4
ffmpeg -i ./records/right.h265 -filter:v fps=10 ./records/right.mp4

