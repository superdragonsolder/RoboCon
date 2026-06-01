#coding=utf-8
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from howlong.berxel.BerxelSdkDriver.BerxelHawkContext import BerxelHawkContext
from howlong.berxel.BerxelSdkDriver.BerxelHawkDefines import BerxelHawkStreamType, BerxelHawkDeviceIntrinsicParams
import numpy as np
import cv2


class CameraManager:
    def __init__(self):
        self.context = None
        self.device = None
        self.device_list = []
        self.intrinsic_params = None
        self._stream_flag = 0

    def initialize(self):
        print("Initializing camera...")
        
        self.context = BerxelHawkContext()
        if self.context is None:
            print("Failed to create camera context")
            return False

        self.context.initCamera()
        
        self.device_list = self.context.getDeviceList()
        if len(self.device_list) < 1:
            print("No device found")
            return False

        print(f"Found device - VID: {self.device_list[0].vendorId}, PID: {self.device_list[0].productId}")
        print(f"Device address: {self.device_list[0].deviceAddress}")
        print(f"Serial number: {self.device_list[0].serialNumber}")

        self.device = self.context.openDevice(self.device_list[0])
        if self.device is None:
            print("Failed to open device")
            return False

        print("Camera initialized successfully")
        return True

    def start_depth_stream(self, denoise=False, with_color=True):
        if self.device is None:
            print("Device not initialized")
            return False

        self.device.setDenoiseStatus(denoise)

        depth_frame_modes = self.device.getSupportFrameModes(
            BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM']
        )
        
        if len(depth_frame_modes) > 0:
            print("Available depth frame modes:")
            for i, mode in enumerate(depth_frame_modes):
                print(f"  [{i}] {mode.resolutionX}x{mode.resolutionY} @ {mode.framerate}fps")
            
            depth_frame_mode = depth_frame_modes[0]
            print(f"Using depth mode: {depth_frame_mode.resolutionX}x{depth_frame_mode.resolutionY} @ {depth_frame_mode.framerate}fps")
        else:
            depth_frame_mode = self.device.getCurrentFrameMode(
                BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM']
            )
        
        if depth_frame_mode is None:
            print("Failed to get depth frame mode")
            return False

        self.device.setFrameMode(
            BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM'],
            depth_frame_mode
        )

        self._stream_flag = BerxelHawkStreamType.forward_dict['BERXEL_HAWK_DEPTH_STREAM']

        color_available = False
        if with_color:
            try:
                color_frame_modes = self.device.getSupportFrameModes(
                    BerxelHawkStreamType.forward_dict['BERXEL_HAWK_COLOR_STREAM']
                )
                if len(color_frame_modes) > 0:
                    print("Available color frame modes:")
                    for i, mode in enumerate(color_frame_modes):
                        print(f"  [{i}] {mode.resolutionX}x{mode.resolutionY} @ {mode.framerate}fps")
                    
                    color_frame_mode = color_frame_modes[0]
                    print(f"Using color mode: {color_frame_mode.resolutionX}x{color_frame_mode.resolutionY} @ {color_frame_mode.framerate}fps")
                    
                    self.device.setFrameMode(
                        BerxelHawkStreamType.forward_dict['BERXEL_HAWK_COLOR_STREAM'],
                        color_frame_mode
                    )
                    
                    self._stream_flag |= BerxelHawkStreamType.forward_dict['BERXEL_HAWK_COLOR_STREAM']
                    color_available = True
                else:
                    print("No color frame modes available")
            except Exception as e:
                print(f"Color stream not available: {e}")

        ret = self.device.startStreams(self._stream_flag)

        if ret != 0:
            print(f"Failed to start streams (error code: {ret})")
            return False

        if color_available:
            print("Color + Depth streams started successfully")
        else:
            print("Depth stream started successfully (no color)")

        try:
            self.intrinsic_params = self.device.getDeviceIntriscParams()
            if self.intrinsic_params:
                print("Camera intrinsic parameters retrieved:")
                print(f"  Color FX: {self.intrinsic_params.colorIntrinsicParams.fx}")
                print(f"  Color FY: {self.intrinsic_params.colorIntrinsicParams.fy}")
                print(f"  Color CX: {self.intrinsic_params.colorIntrinsicParams.cx}")
                print(f"  Color CY: {self.intrinsic_params.colorIntrinsicParams.cy}")
                print(f"  LiteIR FX: {self.intrinsic_params.liteIrIntrinsicParams.fx}")
                print(f"  LiteIR FY: {self.intrinsic_params.liteIrIntrinsicParams.fy}")
                print(f"  LiteIR CX: {self.intrinsic_params.liteIrIntrinsicParams.cx}")
                print(f"  LiteIR CY: {self.intrinsic_params.liteIrIntrinsicParams.cy}")
        except Exception as e:
            print(f"Warning: Could not retrieve intrinsic params: {e}")

        return True

    def get_depth_frame(self, timeout=30):
        if self.device is None:
            return None, None

        hawk_frame = self.device.readDepthFrame(timeout)
        if hawk_frame is None:
            return None, None

        try:
            width = hawk_frame.getWidth()
            height = hawk_frame.getHeight()
            pixel_type = hawk_frame.getPixelType()
            frame_index = hawk_frame.getFrameIndex()
            frame_buffer = hawk_frame.getDataAsUint16()

            if width == 0 or height == 0:
                self.device.releaseFrame(hawk_frame)
                return None, None

            depth_array = np.ndarray(
                shape=(height, width),
                dtype=np.uint16,
                buffer=frame_buffer
            ).copy()

            if frame_index <= 3:
                print(f"Frame {frame_index}: pixel_type={pixel_type:#x}, shape={depth_array.shape}")
                print(f"  min={depth_array.min()}, max={depth_array.max()}, mean={depth_array.mean():.0f}")

            self.device.releaseFrame(hawk_frame)

            return depth_array, pixel_type

        except Exception as e:
            print(f"Error reading depth frame: {e}")
            try:
                self.device.releaseFrame(hawk_frame)
            except:
                pass
            return None, None

    def get_color_frame(self, timeout=30):
        if self.device is None:
            return None

        try:
            hawk_frame = self.device.readColorFrame(timeout)
        except Exception as e:
            return None

        if hawk_frame is None:
            return None

        try:
            width = hawk_frame.getWidth()
            height = hawk_frame.getHeight()
            frame_buffer = hawk_frame.getDataAsUint8()

            if width == 0 or height == 0:
                self.device.releaseFrame(hawk_frame)
                return None

            color_array = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=frame_buffer
            ).copy()

            self.device.releaseFrame(hawk_frame)

            color_bgr = cv2.cvtColor(color_array, cv2.COLOR_RGB2BGR)

            return color_bgr

        except Exception as e:
            try:
                self.device.releaseFrame(hawk_frame)
            except:
                pass
            return None

    def has_color_stream(self):
        return (self._stream_flag & BerxelHawkStreamType.forward_dict['BERXEL_HAWK_COLOR_STREAM']) != 0

    def get_pixel_conversion_factor(self, pixel_type):
        if pixel_type == 0x01:
            return 16.0
        elif pixel_type == 0x02:
            return 7900.0
        else:
            return 7900.0

    def get_camera_info(self):
        if self.device is None:
            return None

        info = {
            'intrinsic_params': self.intrinsic_params,
            'has_color_device': self.device._hasColorDevice if self.device else False,
        }

        try:
            version_info = self.device.getVersion()
            if version_info:
                info['sdk_version'] = f"{version_info.sdkVersion.major}.{version_info.sdkVersion.minor}.{version_info.sdkVersion.revision}"
        except:
            pass

        return info

    def close(self):
        print("Closing camera...")

        if self.device is not None:
            try:
                ret = self.device.stopStream(self._stream_flag)
                if ret == 0:
                    print("Streams closed successfully")
                else:
                    print("Failed to close streams")
            except Exception as e:
                print(f"Error closing stream: {e}")

            try:
                ret = self.context.closeDevice(self.device)
                if ret == 0:
                    print("Device closed successfully")
                else:
                    print("Failed to close device")
            except Exception as e:
                print(f"Error closing device: {e}")

        try:
            if self.context is not None:
                self.context.destroyCamera()
                print("Camera context destroyed")
        except Exception as e:
            print(f"Error destroying camera context: {e}")

        self.device = None
        self.context = None
        print("Camera cleanup complete")
