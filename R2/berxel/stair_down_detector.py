#coding=utf-8
import sys
import os
import time

# 确保 howlong 目录在 sys.path 中（适配行为树导入）
_current_dir = os.path.dirname(os.path.realpath(__file__))
_parent_dir = os.path.dirname(_current_dir)  # howlong/
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

from src.camera_manager import CameraManager
from src.distance_measurer import DistanceMeasurer


class StairDownDetector:
    def __init__(self):
        self.camera = None
        self.measurer = None

        self.roi_width = 300
        self.roi_height = 20
        self.target_distance = 0.57
        self.threshold_ratio = 0.95
        self.tolerance_cm = 0.03

        self.consecutive_frames = 0
        self.required_frames = 5
        self.alert_triggered = False

        self.history = []
        self.max_history = 30

    def initialize(self):
        print("=" * 60)
        print("Berxel Depth Camera - Stair Down Detector")
        print("=" * 60)
        print(f"Configuration:")
        print(f"  ROI Size: {self.roi_width} x {self.roi_height} pixels")
        print(f"  Target Distance: {self.target_distance*100:.0f} cm ({self.target_distance} m)")
        print(f"  Valid Range: {self.target_distance*100:.0f} cm ± {self.tolerance_cm*100:.0f} cm")
        print(f"  Threshold: {self.threshold_ratio*100:.0f}%")
        print(f"  Tolerance: {self.tolerance_cm*100:.0f} cm")
        print(f"  Required Consecutive Frames: {self.required_frames}")
        print("=" * 60)

        self.camera = CameraManager()

        if not self.camera.initialize():
            print("Failed to initialize camera. Exiting...")
            return False

        if not self.camera.start_depth_stream(denoise=False, with_color=False):
            print("Failed to start depth stream. Exiting...")
            self.camera.close()
            return False

        self.measurer = DistanceMeasurer(roi_size=10, temporal_buffer_size=5,
                                        enable_temporal_filter=True)

        print("\nStarting stair down detection...")
        print("Press Ctrl+C to quit")
        print("-" * 60)

        return True

    def is_ready(self) -> bool:
        """供行为树轮询：是否检测到下台阶条件。"""
        return self.alert_triggered

    def reset_detection(self) -> None:
        """重置检测状态（每次下台阶流程启动时调用）。"""
        self.consecutive_frames = 0
        self.alert_triggered = False
        self.history = []

    def close(self) -> None:
        """释放相机资源。"""
        if self.camera is not None:
            try:
                self.camera.close()
            except Exception:
                pass
            self.camera = None
        self.measurer = None

    def check_stair_down(self, depth_frame, pixel_type):
        height, width = depth_frame.shape

        center_x = width // 2
        center_y = height // 2

        half_width = self.roi_width // 2
        half_height = self.roi_height // 2

        x_start = max(0, center_x - half_width)
        x_end = min(width, center_x + half_width)
        y_start = max(0, center_y - half_height)
        y_end = min(height, center_y + half_height)

        roi = depth_frame[y_start:y_end, x_start:x_end]

        distance_array = self.measurer.convert_depth_to_distance(roi, pixel_type)

        valid_distances = distance_array[distance_array > 0]
        total_count = distance_array.size

        if len(valid_distances) == 0:
            return {
                'mean_distance': None,
                'min_distance': None,
                'max_distance': None,
                'valid_count': 0,
                'total_count': total_count,
                'valid_ratio': 0.0
            }

        mean_distance = float(np.mean(valid_distances))
        min_distance = float(np.min(valid_distances))
        max_distance = float(np.max(valid_distances))

        target_min = self.target_distance - self.tolerance_cm
        target_max = self.target_distance + self.tolerance_cm

        valid_count = 0
        for d in valid_distances:
            if target_min <= d <= target_max:
                valid_count += 1

        valid_ratio = valid_count / total_count if total_count > 0 else 0.0

        return {
            'mean_distance': mean_distance,
            'min_distance': min_distance,
            'max_distance': max_distance,
            'valid_count': valid_count,
            'total_count': total_count,
            'valid_ratio': valid_ratio
        }

    def run(self):
        if not self.initialize():
            return

        frame_count = 0
        last_print_time = time.time()

        try:
            while True:
                depth_frame, pixel_type = self.camera.get_depth_frame(timeout=1000)

                if depth_frame is None:
                    print("Warning: Failed to get depth frame")
                    continue

                frame_count += 1

                result = self.check_stair_down(depth_frame, pixel_type)

                if result['mean_distance'] is not None:
                    mean_dist = result['mean_distance']
                    valid_ratio = result['valid_ratio']

                    self.history.append(valid_ratio)
                    if len(self.history) > self.max_history:
                        self.history.pop(0)

                    if valid_ratio >= self.threshold_ratio:
                        self.consecutive_frames += 1
                    else:
                        self.consecutive_frames = 0
                        if self.alert_triggered:
                            print(f"\n>>> Stair down condition no longer met <<<")
                            self.alert_triggered = False

                    if self.consecutive_frames >= self.required_frames and not self.alert_triggered:
                        print(f"\n" + "=" * 60)
                        print(f">>> 可以进行下台阶操作 <<<")
                        print(f">>> CAN GO DOWN STAIR <<<")
                        print(f"=" * 60)
                        print(f"Distance: {mean_dist*100:.2f} cm")
                        print(f"Valid Ratio: {valid_ratio*100:.2f}%")
                        print(f"Consecutive Frames: {self.consecutive_frames}")
                        print(f"=" * 60 + "\n")
                        self.alert_triggered = True

                    current_time = time.time()
                    if current_time - last_print_time >= 1.0:
                        status = "READY" if valid_ratio >= self.threshold_ratio else "NOT READY"
                        alert_flag = " [!ALERT!]" if self.alert_triggered else ""

                        print(f"Frame {frame_count}: "
                              f"Distance = {mean_dist*100:.2f} cm, "
                              f"Valid = {valid_ratio*100:.2f}%, "
                              f"Status = {status}{alert_flag}")

                        last_print_time = current_time
                else:
                    print(f"Frame {frame_count}: No valid depth data")

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        except Exception as e:
            print(f"\n\nError occurred: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("\nCleaning up...")
            if self.camera:
                self.camera.close()
            print("Done.")


def main():
    detector = StairDownDetector()
    detector.run()


if __name__ == '__main__':
    import numpy as np
    main()
