#!/usr/bin/env python
import numpy as np
import cv2
import glob

import rospy
from sensor_msgs.msg import Image,CompressedImage
import cv_bridge
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
print(objp)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('/home/kaushal/sft_match_ws/src/auto/src/3D-pose-estimation-of-a-planar-object/Chess_images/lleft*.jpg')
# cam = cv2.VideoCapture(1)

def calib_cb(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    #bridge = cv_bridge.CvBridge()
    #img = bridge.imgmsg_to_cv2(msg, "bgr8")  

    # ret,img = cam.read()
    # img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
        cv2.namedWindow("Display frame", cv2.WINDOW_NORMAL)
        cv2.imshow("Display frame",img)
        cv2.waitKey(1)
            

        cv2.destroyAllWindows()

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        print(mtx)
        np.savez("BBB", ret=ret, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

        img = cv2.imread('/home/kaushal/sft_match_ws/src/auto/src/3D-pose-estimation-of-a-planar-object/Chess_images/lleft15.jpg')
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        print("New cam matrix is :")
        print(newcameramtx)

        # undistort
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        cv2.imwrite('calibresult.png',dst)

        mean_error = 0
        tot_error = 0
        for i in xrange(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error

        print ("total error: ", mean_error/len(objpoints))
    else:
        print("NO corner")
if __name__=='__main__':
    try:
      rospy.init_node('calib_node', anonymous=True)
      depth_img_sub=rospy.Subscriber("/r200/depth/image_raw4/compressed",CompressedImage,calib_cb)
      rospy.spin()
    except rospy.ROSInterruptException:
      print ("error")
      pass
