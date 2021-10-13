import pyrealsense2 as rs
import cv2 as cv
import numpy as np

pipeline = rs.pipeline()

cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 设定需要对齐的方式（这里是深度对齐彩色，彩色图不变，深度图变换）
align_to = rs.stream.color
# 设定需要对齐的方式（这里是彩色对齐深度，深度图不变，彩色图变换）
# align_to = rs.stream.depth

alignedFs = rs.align(align_to)

profile = pipeline.start(cfg)

try:
    while True:
        fs = pipeline.wait_for_frames()
        aligned_frames = alignedFs.process(fs)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # D·C 191122：打印depth_image的最大值看看
        # print(depth_image.max())
        # 可以看到，最大的数值从一万到六万不等（表示最远距离十多米到六十米这样）

        # D·C 191122：打印数据类型看看
        # print(depth_image.dtype)
        # uint16
        # print(color_image.dtype)
        # uint8

        # D·C 191122：打印color_image的维度看看
        # print(color_image.shape)
        # (480, 640, 3)
        # print(depth_image.shape)
        # (480, 640)

        # D·C 191122：打印cv.convertScaleAbs(depth_image, alpha=0.03)的数据类型和维度看看：
        # print(cv.convertScaleAbs(depth_image, alpha=0.03).dtype)
        # uint8
        # print(cv.convertScaleAbs(depth_image, alpha=0.03).shape)
        # (480, 640)

        # D·C 191122：打印cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)的数据类型和维度看看
        # print(cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET).dtype)
        # uint8
        # print(cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET).shape)
        # (480, 640, 3)

        # D·C 191122：打印cv.convertScaleAbs(depth_image, alpha=0.03)的最大值看看
        # print(cv.convertScaleAbs(depth_image, alpha=0.03))
        # 可看到最大值为255
        # 估计若原值*alpha大于255，则将其取值为255，而当alpha为0.03时，能够映射的最大可变距离为255/0.03=8500mm=8.5m

        # D·C 191122：修改alpha参数后，发现图像对比度发生变化（比如alpha=1，图像基本呈红没啥对比度、alpha=0.001，图像呈蓝也没啥对比度、alpha=1点几，效果也不行）
        # origin：depth_image = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
        depth_image = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.02), cv.COLORMAP_JET)

        images = np.hstack((color_image, depth_image))

        # window = cv.namedWindow('window', cv.WINDOW_AUTOSIZE)

        cv.imshow('window', images)

        cv.waitKey(1)
finally:
    pipeline.stop()

