#2021-10-5
# 主要测试功能： 对齐深度图和RGB图
# 坐标转换
# 测试中心点的xyz坐标

import pyrealsense2 as rs
from scipy.spatial.distance import euclidean
from imutils import perspective
from imutils import contours
import imutils
import numpy as np
import cv2
import json



# 通过RGB图像中的两个点，计算出他们之间的距离
def get_3d_coordinates(color_intr,depth_frame,xpix1,ypix1,xpix2,ypix2):
	dist1 = depth_frame.get_distance(xpix1,ypix1)
	dist2 = depth_frame.get_distance(xpix2,ypix2)

    #Given pixel coordinates and depth in an image 
    #with no distortion or inverse distortion coefficients, 
    # compute the corresponding point in 3D space relative to the same camera
    # 计算出空间点
	depth_point1 = rs.rs2_deproject_pixel_to_point(color_intr,[xpix1,ypix1],dist1)
	depth_point2 = rs.rs2_deproject_pixel_to_point(color_intr,[xpix2,ypix2],dist2)

	return np.squrt(np.power(depth_point1[0]-depth_point2[0],2) +
		np.power(depth_point1[1]-depth_point2[1],2)+
		np.power(depth_point2[2]-depth_point2[2],2)
	)


def show_images(images):
	for i, img in enumerate(images):
		cv2.imshow("image_" + str(i), img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def order_points(pts):
    # pts为轮廓坐标
    # 返回列表中存储元素分别为左上角，右上角，右下角和左下角
    # 这个函数是为了更便利的计算面积
    rect = np.zeros((4, 2), dtype = "float32")
    # 左上角的点具有最小的和，而右下角的点具有最大的和
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    # 计算点之间的差值
    # 右上角的点具有最小的差值,
    # 左下角的点具有最大的差值
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    # 返回排序坐标(依次为左上右上右下左下)
    return rect

def get_aligned_images():
    print("Get aligned images")
    frames = pipeline.wait_for_frames()  #等待获取图像帧
    aligned_frames = align.process(frames)  #获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  #获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()   #获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics   #获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  #获取深度参数（像素坐标系转相机坐标系会用到）
    camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }
    # 保存内参到本地
    with open('./intrinsics.json', 'w') as fp:
        json.dump(camera_parameters, fp)
    #######################################################
    
    depth_image = np.asanyarray(aligned_depth_frame.get_data())  #深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  #深度图（8位）
    depth_image_3d = np.dstack((depth_image_8bit,depth_image_8bit,depth_image_8bit))  #3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图
    
    #返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    print("Return aligned results")
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame





if __name__ == "__main__":
    #  Init 初始化
    pipeline = rs.pipeline()  #定义流程pipeline
    config = rs.config()   #定义配置config
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  #配置depth流
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)   #配置color流
    profile = pipeline.start(config)  #流程开始
    align_to = rs.stream.color  #与color流对齐
    align = rs.align(align_to)
    print("Open RealSense Device ")
    while 1:
        intr, depth_intrin, rgb, depth, aligned_depth_frame = get_aligned_images() #获取对齐的图像与相机内参
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Gray",gray)
        # 高斯模糊，不然canny查不到轮廓
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        #cv2.imshow("blur", blur)

        #以下为opencv中的形态学处理

        #Canny函数为非微分边缘检测算法，具有滤波、增强、检测的多阶段优化算子。
        edged = cv2.Canny(blur, 50, 100)

        #图像膨胀的用途：
        #用途1：Dilation 影像膨脹通常是配合著影像侵蝕 Erosion 使用，先使用侵蝕的方式使影像中的線條變窄，同時也去除雜訊，之後再透過 Dilation 將影像膨脹回來。
        #用途2：用來連接兩個很靠近但分開的物體。
        edged = cv2.dilate(edged, None, iterations=1) # 图像膨胀


        #图像腐蚀Erosion的用途：
        #用途1：Erosion 影像侵蝕對於移除影像中的小白雜點很有幫助，可用來去噪，例如影像中的小雜點，雜訊。
        #用途2：細化影像，消除毛刺。
        edged = cv2.erode(edged, None, iterations=1) #图像腐蚀

        cv2.imshow("Canny", blur)

        #显示所有的过程图片，测试时，可以选择用此句
        #show_images([blur, edged])

        # Find contours
        # 寻找边缘
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # Sort contours from left to right as leftmost contour is reference object
        #(cnts, _) = contours.sort_contours(cnts)

        # Remove contours which are not large enough
        # 做一个阈值滤波，如果尺寸小于某个范围的就排除
        cnts = [x for x in cnts if cv2.contourArea(x) > 400]

        # Draw contours
        # 画出边缘
        cv2.drawContours(rgb, cnts, -1, (0,250,0), 2)

        #show_images([blur, edged])
                #print(len(cnts))

        # Find corners 
        # 查找各种角
        corners = cv2.goodFeaturesToTrack(gray,25,0.01,10)
        corners = np.int0(corners)
        for i in corners :
        	x,y = i.ravel()
        	cv2.circle(rgb,(x,y),3,255,-1)
        cv2.imshow("CornersFinder",rgb)
        '''
        c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]

        # 画四个边缘点

        rect = order_points(c.reshape(c.shape[0], 2))
        #print(rect)


        xs = [i[0] for i in rect]
        ys = [i[1] for i in rect]
        xs.sort()
        ys.sort()
        #内接矩形的坐标为
        #print(xs,ys)
        #print(xs[1],xs[2],ys[1],ys[2])
        print(type(rect[0]))
        print(rect[0])
        rgb = cv2.circle(rgb,tuple(rect[0]),radius=6, color=(255,0,255),thickness=2)
        rgb = cv2.circle(rgb,tuple(rect[1]),radius=6, color=(255,0,255),thickness=2)
        rgb = cv2.circle(rgb,tuple(rect[2]),radius=6, color=(255,0,255),thickness=2)
        rgb = cv2.circle(rgb,tuple(rect[3]),radius=6, color=(255,0,255),thickness=2)


        # Reference object dimensions
        # Here for reference I have used a 2cm x 2cm square
        # 设置一个参考矩形用作Marker和基础尺寸，算出实际尺寸与像素尺寸比。
        # 左上角放置一个正方形
        ref_object = cnts[0]
        box = cv2.minAreaRect(ref_object)
        box = cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
        (tl, tr, br, bl) = box
        dist_in_pixel = euclidean(tl, tr)
        dist_in_cm = 2
        pixel_per_cm = dist_in_pixel/dist_in_cm



        # 画剩下的轮廓
        for cnt in cnts:
        	#print(cnt[0])
        	#print(cnt[0][0][0])
        	#print(cnt[0][0][1])
            #画白圈
        	leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
        	rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
        	topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
        	bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
        	rgb = cv2.circle(rgb,leftmost,radius=6, color=(255,255,255),thickness=2)
        	rgb = cv2.circle(rgb,rightmost,radius=6, color=(255,255,255),thickness=2)
        	rgb = cv2.circle(rgb,topmost,radius=6, color=(255,255,255),thickness=2)
        	rgb = cv2.circle(rgb,bottommost,radius=6, color=(255,255,255),thickness=2)
        	#image = cv2.circle(image,cnt[0],radius=6, color=(255,255,255),thickness=2)
        	cv2.imshow("circle",rgb)


        	box = cv2.minAreaRect(cnt)
        	box = cv2.boxPoints(box)
        	box = np.array(box, dtype="int")

        	box = perspective.order_points(box)


        	(tl, tr, br, bl) = box
        	cv2.drawContours(rgb, [box.astype("int")], -1, (0, 0, 255), 2)
        	mid_pt_horizontal = (tl[0] + int(abs(tr[0] - tl[0])/2), tl[1] + int(abs(tr[1] - tl[1])/2))
        	mid_pt_verticle = (tr[0] + int(abs(tr[0] - br[0])/2), tr[1] + int(abs(tr[1] - br[1])/2))
        	wid = euclidean(tl, tr)/pixel_per_cm
        	ht = euclidean(tr, br)/pixel_per_cm
        	cv2.putText(rgb, "{:.1f}cm".format(wid), (int(mid_pt_horizontal[0] - 15), int(mid_pt_horizontal[1] - 10)), 
        		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        	cv2.putText(rgb, "{:.1f}cm".format(ht), (int(mid_pt_verticle[0] + 10), int(mid_pt_verticle[1])), 
        		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        show_images([rgb])

'''

'''

        # 定义需要得到真实三维信息的像素点（x, y)，本例程以中心点为例
        # 测试精度，以Z轴为主要考量因素。
        x = 320  
        y = 240
        dis = aligned_depth_frame.get_distance(x, y)  #（x, y)点的真实深度值
        camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], dis)  #（x, y)点在相机坐标系下的真实值，为一个三维向量。其中camera_coordinate[2]仍为dis，camera_coordinate[0]和camera_coordinate[1]为相机坐标系下的xy真实距离。
        print(camera_coordinate) #打印出xyz坐标
        #画了一个小圈圈
        rgb = cv2.circle(rgb,(320,240),radius=6, color=(255,255,255),thickness=2)
        cv2.imshow('RGB image',rgb)  #显示彩色图
        key = cv2.waitKey(1)
        # ESC 和 q 键 退出程序
        if key & 0xFF == ord('q') or key == 27:
            pipeline.stop()
            break
        cv2.destroyAllWindows()
'''