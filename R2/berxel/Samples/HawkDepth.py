#coding=utf-8
import struct
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append('../BerxelSdkDriver/')


from BerxelSdkDriver.BerxelHawkDefines import *
from BerxelSdkDriver.BerxelHawkFrame import *
from BerxelSdkDriver.BerxelHawkDevice import *
from BerxelSdkDriver.BerxelHawkContext import *




import time
import datetime
import threading

import numpy as np
import cv2


fx = 0.0
fy = 0.0
cx = 0.0
cy = 0.0

bFirst = True

class HawkDepth(object):

    def __init__(self):
        self.__context = None
        self.__device = None
        self.__deviceList = []

    # Step 1 open device
    def openDevice(self):



        self.__context = BerxelHawkContext()

        if self.__context is None:
            print("init failed")
            return  False

        self.__context.initCamera()

        self.__deviceList = self.__context.getDeviceList()

        if len(self.__deviceList) <  1:
            print("can not find device")
            return False

        print("china 1")

        print("vid : ", self.__deviceList[0].vendorId)
        print("pid : ", self.__deviceList[0].productId)
        print("device addres : ", self.__deviceList[0].deviceAddress)

        print("china 2")
        self.__device = self.__context.openDevice(self.__deviceList[0])

        if self.__device is None:
            return False

        return True

    # Step 2 ： open Stream

    def startStream(self):

        if self.__device is None:
            return  False

        #self.__device.setRegistrationEnable(True)
        #self.__device.setFrameSync(True)
        #self.__device.setSystemClock()

        self.__device.setDenoiseStatus(False)





        frameMode = self.__device.getCurrentFrameMode(BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM'])
        self.__device.setFrameMode(BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM'] ,frameMode)

        ret = self.__device.startStreams(BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM'])

        global fx, fy, cx, cy

        # 获取的是1280 * 800的相机内参，若是需要640的，可以除以2.
        # 下列代码获取的是未配准的深度相机的内参，若深度图配准后，相机内参应该使用彩色相机内参 intrinsicParams.colorIntrinsicParams

        intrinsicParams = BerxelHawkDeviceIntrinsicParams()
        intrinsicParams = self.__device.getDeviceIntriscParams()

        print(intrinsicParams.colorIntrinsicParams.fx)
        print(intrinsicParams.colorIntrinsicParams.fy)
        print(intrinsicParams.colorIntrinsicParams.cx)
        print(intrinsicParams.colorIntrinsicParams.cy)

        print(intrinsicParams.liteIrIntrinsicParams.fx)
        print(intrinsicParams.liteIrIntrinsicParams.fy)
        print(intrinsicParams.liteIrIntrinsicParams.cx)
        print(intrinsicParams.liteIrIntrinsicParams.cy)


        print("rotaion: ")

        print(intrinsicParams.rotateIntrinsicParams.rotate1)
        print(intrinsicParams.rotateIntrinsicParams.rotate2)
        print(intrinsicParams.rotateIntrinsicParams.rotate3)
        print(intrinsicParams.rotateIntrinsicParams.rotate4)
        print(intrinsicParams.rotateIntrinsicParams.rotate5)
        print(intrinsicParams.rotateIntrinsicParams.rotate6)
        print(intrinsicParams.rotateIntrinsicParams.rotate7)
        print(intrinsicParams.rotateIntrinsicParams.rotate8)
        print(intrinsicParams.rotateIntrinsicParams.rotate9)

        print("translation: ")
        print(intrinsicParams.translationIntrinsicParams.translation1)
        print(intrinsicParams.translationIntrinsicParams.translation2)
        print(intrinsicParams.translationIntrinsicParams.translation3)



        if ret == 0:
            print("start stream succeed")
            return True
        else:
            print("start stream failed")
            return False



    # 3: read Frame

    def displayImage(self):

        hawkFrame = self.__device.readDepthFrame(30)
        if hawkFrame is None:
            return 1

        width  = hawkFrame.getWidth()
        height = hawkFrame.getHeight()
        streamType = hawkFrame.getStreamType()
        dataSize = hawkFrame.getDataSize()
        pxielType = hawkFrame.getPixelType()
        index = hawkFrame.getFrameIndex()
        frameBuffer = hawkFrame.getDataAsUint16()

        # print("pixel = ", pxielType)
        # print("type = ", streamType)
        # print("frameIndex = ", index)
        # print("fps =", hawkFrame.getFps())
        # print("w= ", width)
        # print("h =", height)
        # print("datasize = ", dataSize)
        # print("stamp = ", hawkFrame.getTimeStamp())
        # print(frameBuffer)



        #global fx, fy, cx, cy,bFirst
        #转换点云
        #if bFirst == True:
        #    bFirst = False
        #    tempPoint3DList = self.__device.converDepthToPoint(hawkFrame.getOriData(), width, height, 1000.0, 841.507 /2, 841.507 /2, 636.436/2, 404.947/2, pxielType)
        #    for i in range(640 * 400):
        #        print("i , x y z = " , i,  tempPoint3DList.point3DList[i].x, tempPoint3DList.point3DList[i].y, tempPoint3DList.point3DList[i].z)
        #    print("New")

        depth_array = np.ndarray(shape=(height, width), dtype=np.uint16, buffer=frameBuffer)
        depth_array_disp = ((depth_array / 10000.) * 255).astype(np.uint8)

        cv2.namedWindow("Depth", cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Depth', np.uint8(depth_array_disp))

        ch = 0xFF & cv2.waitKey(1)

        if ch == 27 or ch == 81 or ch == 113:
            return -1

        self.__device.releaseFrame(hawkFrame)

        return 1

    def ShowFrame(self):

        print ("Sart Show Frame...")

        time.sleep(1)
        while 1:
            ret = self.displayImage()
            if ret != 1:
                break

        self.closeStream()
        self.closeDevice()
        return




    #4 : closeStream
    def closeStream(self):
        if self.__device is None:
            return  False

        ret = self.__device.stopStream(BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM'])

        if ret == 0:
            print("close stream succeed")
            return True
        else:
            print("clsoe stream failed")
            return False

    def closeDevice(self):
        if self.__context is None:
            return False

        if self.__device is None:
            return  False

        ret = self.__context.closeDevice(self.__device)
        if ret == 0:
            print("clsoe device succeed")
        else:
            print("close device Failed")

        self.__context.destroyCamera()


    def StartTest(self):

        if self.openDevice() == False:
            return
        #
        if self.startStream() == False:
            return

        tShowFrame = threading.Thread(target=self.ShowFrame)
        tShowFrame.start()




if __name__ == '__main__':
    print('PyCharm')
    depthView = HawkDepth()
    depthView.StartTest()
