#!/usr/bin/env python3
# https://docs.luxonis.com/projects/api/en/latest/samples/VideoEncoder/rgb_mono_encoding/

import depthai as dai
import rosbag
from sensor_msgs.msg import Imu

# Create pipeline
pipeline = dai.Pipeline()


# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
imu = pipeline.create(dai.node.IMU)

manipISP = pipeline.create(dai.node.ImageManip)
manipPreview = pipeline.create(dai.node.ImageManip)

encLeft = pipeline.create(dai.node.VideoEncoder)
encRight = pipeline.create(dai.node.VideoEncoder)
encRgb = pipeline.create(dai.node.VideoEncoder)
encPreview = pipeline.create(dai.node.VideoEncoder)

encLeftOut = pipeline.create(dai.node.XLinkOut)
encRightOut = pipeline.create(dai.node.XLinkOut)
encRgbOut = pipeline.create(dai.node.XLinkOut)
encPreviewOut = pipeline.create(dai.node.XLinkOut)
imuOut = pipeline.create(dai.node.XLinkOut)

encLeftOut.setStreamName('encLeftOut')
encRightOut.setStreamName('encRightOut')
encRgbOut.setStreamName('encRgbOut')
encPreviewOut.setStreamName('encPreviewOut')
imuOut.setStreamName('imu')

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setPreviewKeepAspectRatio(True)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setInterleaved(False)
camRgb.setPreviewSize(416,416)
camRgb.setFps(30)
camRgb.setIspScale(2, 3)

monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoLeft.setFps(30)

monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setFps(30)

manipISP.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
manipISP.setMaxOutputFrameSize(10000000)
manipPreview.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
manipPreview.setMaxOutputFrameSize(10000000)

imu.enableIMUSensor([dai.IMUSensor.GYROSCOPE_RAW, dai.IMUSensor.ACCELEROMETER_RAW], 100)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)

# Create encoders, one for each camera, consuming the frames and encoding them using H.264 / H.265 encoding
encLeft.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
encRight.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
encRgb.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
encPreview.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)

# Linking
monoLeft.out.link(encLeft.input)
monoRight.out.link(encRight.input)
camRgb.isp.link(manipISP.inputImage)
manipISP.out.link(encRgb.input)
camRgb.preview.link(manipPreview.inputImage)
manipPreview.out.link(encPreview.input)
imu.out.link(imuOut.input)

encLeft.bitstream.link(encLeftOut.input)
encRight.bitstream.link(encRightOut.input)
encRgb.bitstream.link(encRgbOut.input)
encPreview.bitstream.link(encPreviewOut.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as dev:

    # Output queues will be used to get the encoded data from the outputs defined above
    outLeft    = dev.getOutputQueue(name='encLeftOut'   , maxSize=30, blocking=False)
    outRight   = dev.getOutputQueue(name='encRightOut'  , maxSize=30, blocking=False)
    outRgb     = dev.getOutputQueue(name='encRgbOut'    , maxSize=30, blocking=False)
    outPreview = dev.getOutputQueue(name='encPreviewOut', maxSize=30, blocking=False)
    outImu     = dev.getOutputQueue(name='imu'          , maxSize=30, blocking=False)

    with rosbag.Bag('test.bag', 'w') as bag, open('records/right.h265', 'wb') as fileRightH265, open('records/left.h265', 'wb') as fileLeftH265, open('records/rgb.h265', 'wb') as fileColorH265, open('records/preview.h265', 'wb') as filePreviewH265:
        print("Press Ctrl+C to stop encoding...")
        while True:
            try:
                # Empty each queue
                while outLeft.has():
                    data = outLeft.get().getData().tofile(fileLeftH265)

                while outRight.has():
                    outRight.get().getData().tofile(fileRightH265)

                while outRgb.has():
                    outRgb.get().getData().tofile(fileColorH265)

                while outPreview.has():
                    outPreview.get().getData().tofile(filePreviewH265)

                while outPreview.has():
                    outPreview.get().getData().tofile(filePreviewH265)

                if outImu.has():
                    try:
                        packet = outImu.get().getRaw().packets.pop()

                        data = dai.RawIMUData()
                        # data.packets[0].acceleroMeter.timestamp.
                        
                            
                        #     print(buffer.data)

                        msg = Imu()
                        msg.linear_acceleration.x = packet.
                        msg.linear_acceleration.y =
                        msg.linear_acceleration.z =
                        #     bag.write('imu', msg)

                        # except Exception:
                        #     print(Exception)

                    except IndexError as e:
            

            except KeyboardInterrupt:
                # Keyboard interrupt (Ctrl + C) detected
                bag.close()
                break

    print("To view the encoded data, convert the stream file (.h264/.h265) into a video file (.mp4), using command below:")
    print("./convert.bash")
