# Base image
# FROM nvidia/cuda:12.1.1-cudnn8-runtime-ubuntu20.04
FROM ubuntu:20.04

WORKDIR /app

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && \
    apt-get install -yq --no-install-recommends git python3 python3-pip python3-opencv && \ 
    git clone https://github.com/ultralytics/yolov5.git /app/yolov5 && \
    cd /app/yolov5 && \
    pip3 install -U -r requirements.txt

# Set the working directory to /app/yolov5
WORKDIR /app/yolov5

# Train the model on the dataset using the nvidia runtime
CMD ["python3", "train.py", "--img", "640", "--batch", "32", "--epochs", "1", "--data", "/app/dataset/StrawberryRedGreenFlower/Strawberry/data.yaml", "--weights", "yolov5s.pt", "--device", "cpu"]
