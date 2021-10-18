import numpy as np
import pyrealsense2 as rs
from scipy.spatial import distance as dist


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

def get_3d_coordinates(color_intr,depth_frame,xpix1,ypix1,xpix2,ypix2):
	dist1 = depth_frame.get_distance(xpix1,ypix1)
	dist2 = depth_frame.get_distance(xpix2,ypix2)

    #Given pixel coordinates and depth in an image 
    #with no distortion or inverse distortion coefficients, 
    # compute the corresponding point in 3D space relative to the same camera
    # 计算出空间点
	depth_point1 = rs.rs2_deproject_pixel_to_point(color_intr,[xpix1,ypix1],dist1)
	depth_point2 = rs.rs2_deproject_pixel_to_point(color_intr,[xpix2,ypix2],dist2)

	return np.sqrt(np.power(depth_point1[0]-depth_point2[0],2) +
		np.power(depth_point1[1]-depth_point2[1],2)+
		np.power(depth_point2[2]-depth_point2[2],2)
	)

def get_distances(tltrX, tltrY, blbrX, blbrY, tlblX, tlblY, trbrX, trbrY):
    dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
    dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
    return dA, dB