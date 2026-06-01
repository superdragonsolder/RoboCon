#coding=utf-8
from .BerxelHawkDefines import *

import ctypes.util
import platform
import os.path
import sys

from ctypes import *


def onloadLibrary():

    dirname, filename = os.path.split(os.path.abspath(__file__))
    system = platform.system()
    print("dirname = ", dirname)
    print("filename = " ,filename)
    if system == 'Windows':
        dll_loader = ctypes.CDLL
        commom_dll = os.path.abspath(dirname + '/libs/windows/BerxelInterface.dll')
        #commom_dll = ('../../lib/x64/BerxelInterface.dll')
    else:
        dll_loader = ctypes.CDLL
        commom_so = os.path.abspath(dirname + '/libs/linux/libBerxelInterface.so') #so路径在Linux上自行配置
    try:
        if system == 'Windows':
            return ctypes.LibraryLoader(dll_loader).LoadLibrary(commom_dll)
        else:
            return ctypes.LibraryLoader(dll_loader).LoadLibrary(commom_so)
    except OSError:
        return None





berxelDll = onloadLibrary()

berxelInit = berxelDll.berxelInit
berxelInit.restype = c_int

berxelDestroy = berxelDll.berxelDestroy
berxelDestroy.restype = c_int

berxelGetDeviceList = berxelDll.berxelGetDeviceList
berxelGetDeviceList.argtypes = [deviceInfoHandle_p,POINTER(c_uint32)]
berxelGetDeviceList.restype = c_int


berxelReleaseDeviceList = berxelDll.berxelReleaseDeviceList
berxelReleaseDeviceList.argtypes = [deviceInfoHandle_p]
berxelReleaseDeviceList.restype = c_int



berxelOpenDeviceByAddr = berxelDll.berxelOpenDeviceByAddr
berxelOpenDeviceByAddr.argtypes = [c_char_p, deviceHandle_p]
berxelOpenDeviceByAddr.restype = c_int



berxelCloseDevice = berxelDll.berxelCloseDevice
berxelCloseDevice.argtypes = [deviceHandle]
berxelCloseDevice.restype = c_int


berxelGetVersion = berxelDll.berxelGetVersion
berxelGetVersion.argtypes = [deviceHandle, versionHandle]
berxelGetVersion.restype = c_int

berxelGetCurrentDeviceInfo = berxelDll.berxelGetCurrentDeviceInfo
berxelGetCurrentDeviceInfo.argtypes = [deviceHandle, deviceInfoHandle]
berxelGetCurrentDeviceInfo.restype = c_int


berxelGetDeviceIntriscParams = berxelDll.berxelGetDeviceIntriscParams
berxelGetDeviceIntriscParams.argtypes = [deviceHandle, deviceIntrinsicParamsHandle]
berxelGetDeviceIntriscParams.restype = c_int


berxelSetStreamMirror = berxelDll.berxelSetStreamMirror
berxelSetStreamMirror.argtypes = [deviceHandle, c_uint32]
berxelSetStreamMirror.restype = c_int


berxelSetDeviceStatusCallback = berxelDll.berxelSetDeviceStatusCallback
berxelSetDeviceStatusCallback.argtypes = [BerxelDeviceStatusCallback, c_void_p]
berxelSetDeviceStatusCallback.restype = c_int


berxelSetStreamFlagMode = berxelDll.berxelSetStreamFlagMode
berxelSetStreamFlagMode.argtypes = [deviceHandle, c_uint32]
berxelSetStreamFlagMode.restype = c_int


berxelEnableRegistration = berxelDll.berxelEnableRegistration
berxelEnableRegistration.argtypes = [deviceHandle, c_uint32]
berxelEnableRegistration.restype = c_int

berxelGetSupportStreamFrameMode = berxelDll.berxelGetSupportStreamFrameMode
berxelGetSupportStreamFrameMode.argtypes = [deviceHandle, c_uint32, frameModeHandle_p, POINTER(c_uint32)]
berxelGetSupportStreamFrameMode.restype = c_int


berxelSetStreamFrameMode = berxelDll.berxelSetStreamFrameMode
berxelSetStreamFrameMode.argtypes = [deviceHandle, c_uint32,  frameModeHandle]
berxelSetStreamFrameMode.restype = c_int

berxelGetCurrentStramFrameMode = berxelDll.berxelGetCurrentStramFrameMode
berxelGetCurrentStramFrameMode.argtypes = [deviceHandle, c_uint32]
berxelGetCurrentStramFrameMode.restype = frameModeHandle

berxelOpenStream = berxelDll.berxelOpenStream
berxelOpenStream.argtypes = [deviceHandle, c_uint32, streamHandle_p]
berxelOpenStream.restype = c_int

berxelOpenStream2 = berxelDll.berxelOpenStream2
berxelOpenStream2.argtypes = [deviceHandle, c_uint32, streamHandle_p,  BerxelNewFrameCallback, c_void_p]
berxelOpenStream2.restype = c_int

berxelCloseStream = berxelDll.berxelCloseStream
berxelCloseStream.argtypes = [streamHandle]
berxelCloseStream.restype = c_int

berxelReadFrame = berxelDll.berxelReadFrame
berxelReadFrame.argtypes = [streamHandle, imageFrameHandle_p, c_int32]
berxelReadFrame.restype = c_int


berxelReleaseFrame = berxelDll.berxelReleaseFrame
berxelReleaseFrame.argtypes = [imageFrameHandle_p]
berxelReleaseFrame.restype = c_int

berxelSetFrameSync = berxelDll.berxelSetFrameSync
berxelSetFrameSync.argtypes = [deviceHandle, c_uint32]
berxelSetFrameSync.restype = c_int

berxelSetSafetyMode = berxelDll.berxelSetSafetyMode
berxelSetSafetyMode.argtypes = [deviceHandle, c_uint32]
berxelSetSafetyMode.restype = c_int


berxelSetSystemClock = berxelDll.berxelSetSystemClock
berxelSetSystemClock.argtypes = [deviceHandle]
berxelSetSystemClock.restype = c_int

berxelSetDenoise = berxelDll.berxelSetDenoise
berxelSetDenoise.argtypes = [deviceHandle, c_uint32]
berxelSetDenoise.restype = c_int

berxelSetColorQuality = berxelDll.berxelSetColorQuality
berxelSetColorQuality.argtypes = [deviceHandle, c_uint32]
berxelSetColorQuality.restype = c_int

#BERXEL_EXPOPRT int32_t berxelConvertDepthToPointCloud(const uint16_t* pDepth, uint32_t width, uint32_t height, float factor, float fx, float fy, float cx, float cy, BerxelPoint3D* pPointClouds, BerxelPixelFormat format);
berxelConvertDepthToPointCloud = berxelDll.berxelConvertDepthToPointCloud
berxelConvertDepthToPointCloud.argtypes = [c_void_p, c_uint32,c_uint32 ,c_float,c_float,c_float,c_float,c_float,pointListHandle,c_uint32]
berxelConvertDepthToPointCloud.restype = c_int


berxelSetColorExposureGain = berxelDll.berxelSetColorExposureGain
berxelSetColorExposureGain.argtypes  = [deviceHandle, c_uint32, c_uint32]
berxelSetColorExposureGain.restype   = c_int

berxelRecoveryColorAE = berxelDll.berxelRecoveryColorAE
berxelRecoveryColorAE.argtypes  = [deviceHandle]
berxelRecoveryColorAE.restype   = c_int

berxelEnableTemporalDenoise = berxelDll.berxelEnableTemporalDenoise
berxelEnableTemporalDenoise.argtypes = [deviceHandle, c_uint32]
berxelEnableTemporalDenoise.restype = c_int

berxelEnableSpatialDenoise = berxelDll.berxelEnableSpatialDenoise
berxelEnableSpatialDenoise.argtypes = [deviceHandle, c_uint32]
berxelEnableSpatialDenoise.restype = c_int

berxelSetDepthElectricCurrent = berxelDll.berxelSetDepthElectricCurrent
berxelSetDepthElectricCurrent.argtypes = [deviceHandle, c_uint32]
berxelSetDepthElectricCurrent.restype = c_int

