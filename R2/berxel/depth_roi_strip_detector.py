#coding=utf-8
import sys
import os
import time
import cv2

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from howlong.berxel.src.camera_manager import CameraManager
from howlong.berxel.src.distance_measurer import DistanceMeasurer
from howlong.berxel.src.visualizer import Visualizer


class StripDetectorApp:
    def __init__(self):
        self.camera = None
        self.measurer = None
        self.visualizer = None

        self.strip_width = 300
        self.strip_height = 20
        self.vertical = True
        self.y_position = None
        self.threshold_ratio = 0.5
        self.tolerance_percent = 5.0

        self.ratio_history = []
        self.max_history = 100

        self.window_name = "Depth Strip ROI Detector"

    def setup_trackbars(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        cv2.createTrackbar('Threshold (%)', self.window_name, 50, 100,
                          self.on_threshold_change)
        cv2.createTrackbar('Strip Width', self.window_name, 300, 400,
                          self.on_strip_width_change)
        cv2.createTrackbar('Strip Height', self.window_name, 20, 100,
                          self.on_strip_height_change)
        cv2.createTrackbar('Y Position (%)', self.window_name, 50, 100,
                          self.on_y_position_change)
        cv2.createTrackbar('Mode (0=H/1=V)', self.window_name, 1, 1,
                          self.on_mode_change)
        cv2.createTrackbar('Tolerance (%)', self.window_name, 5, 20,
                          self.on_tolerance_change)

    def on_threshold_change(self, value):
        self.threshold_ratio = value / 100.0

    def on_strip_width_change(self, value):
        self.strip_width = max(10, value)

    def on_strip_height_change(self, value):
        self.strip_height = max(5, value)

    def on_y_position_change(self, value):
        self.y_position = int(value / 100.0 * 480)

    def on_mode_change(self, value):
        self.vertical = (value == 1)

    def on_tolerance_change(self, value):
        self.tolerance_percent = max(1, value)

    def initialize(self):
        print("=" * 60)
        print("Berxel Depth Camera - Strip ROI Detector")
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

        self.visualizer = Visualizer(window_name=self.window_name)

        self.setup_trackbars()

        print("\nStarting strip ROI detection...")
        print("Press 'q' or 'ESC' to quit")
        print("-" * 60)

        return True

    def run(self):
        if not self.initialize():
            return

        frame_count = 0
        fps_counter = 0
        last_frame_time = time.time()
        fps_display = 0

        try:
            while True:
                depth_frame, pixel_type = self.camera.get_depth_frame(timeout=1000)

                if depth_frame is None:
                    print("Warning: Failed to get depth frame")
                    continue

                frame_count += 1
                fps_counter += 1

                current_time = time.time()
                elapsed = current_time - last_frame_time
                if elapsed >= 1.0:
                    fps_display = fps_counter / elapsed
                    fps_counter = 0
                    last_frame_time = current_time

                height_analysis = self.measurer.analyze_height_distribution(
                    depth_frame, pixel_type,
                    strip_width=self.strip_width,
                    strip_height=self.strip_height,
                    vertical=self.vertical,
                    y_position=self.y_position,
                    tolerance_percent=self.tolerance_percent
                )

                distance = height_analysis['distance']
                valid_count = height_analysis['valid_count']
                total_count = height_analysis['total_count']
                valid_ratio = height_analysis['valid_ratio']

                height_levels = height_analysis['height_levels']
                level_percentages = height_analysis['level_percentages']
                num_different_heights = height_analysis['num_different_heights']
                is_uniform = height_analysis['is_uniform']

                self.ratio_history.append(valid_ratio)
                if len(self.ratio_history) > self.max_history:
                    self.ratio_history.pop(0)

                conversion_factor = 7900.0 if pixel_type == 0x02 else 16.0
                depth_display = self.visualizer.create_depth_display(
                    depth_frame, pixel_type, conversion_factor)

                depth_display = self.visualizer.draw_strip_roi(
                    depth_display,
                    strip_width=self.strip_width,
                    strip_height=self.strip_height,
                    vertical=self.vertical,
                    y_position=self.y_position,
                    valid_ratio=valid_ratio,
                    distance=distance
                )

                height, width = depth_display.shape[:2]
                info_panel_y = height - 140
                panel_color = (40, 40, 40)
                overlay = depth_display.copy()
                cv2.rectangle(overlay, (0, info_panel_y - 20),
                             (width, height), panel_color, -1)
                cv2.addWeighted(overlay, 0.7, depth_display, 0.3, 0, depth_display)

                y_pos = info_panel_y
                font = cv2.FONT_HERSHEY_SIMPLEX

                pixel_type_str = "12I_4D" if pixel_type == 0x01 else "13I_3D" if pixel_type == 0x02 else "Unknown"
                cv2.putText(depth_display, f"Pixel Type: {pixel_type_str}",
                           (10, y_pos), font, 0.4, (255, 255, 255), 1)
                y_pos += 20

                mode_str = "Vertical" if self.vertical else "Horizontal"
                cv2.putText(depth_display, f"Mode: {mode_str}  Strip: {self.strip_width}x{self.strip_height}",
                           (10, y_pos), font, 0.4, (200, 200, 200), 1)
                y_pos += 20

                if distance is not None:
                    distance_text = f"Distance: {distance*100:.2f} cm"
                    color = (0, 255, 0)
                else:
                    distance_text = "Distance: N/A"
                    color = (0, 0, 255)
                cv2.putText(depth_display, distance_text,
                           (10, y_pos), font, 0.5, color, 1)
                y_pos += 20

                ratio_text = f"Valid Ratio: {valid_ratio*100:.2f}%  Threshold: {self.threshold_ratio*100:.0f}%"
                ratio_color = (0, 255, 0) if valid_ratio >= self.threshold_ratio else (0, 128, 255)
                cv2.putText(depth_display, ratio_text,
                           (10, y_pos), font, 0.5, ratio_color, 1)
                y_pos += 20

                height_info = f"Height Levels: {num_different_heights}  Tolerance: {self.tolerance_percent}%"
                if is_uniform:
                    height_color = (0, 255, 0)
                else:
                    height_color = (0, 200, 255)
                cv2.putText(depth_display, height_info,
                           (10, y_pos), font, 0.4, height_color, 1)
                y_pos += 18

                if num_different_heights > 0 and num_different_heights <= 5:
                    display_text = f"H1={height_levels[0]:.1f}cm:{level_percentages[0]:.1f}%"
                    cv2.putText(depth_display, display_text,
                               (10, y_pos), font, 0.35, (200, 200, 200), 1)
                    y_pos += 16
                    for i in range(1, len(height_levels)):
                        display_text = f"H{i+1}={height_levels[i]:.1f}cm:{level_percentages[i]:.1f}%"
                        cv2.putText(depth_display, display_text,
                                   (10, y_pos), font, 0.35, (200, 200, 200), 1)
                        y_pos += 16

                if valid_ratio >= self.threshold_ratio:
                    alarm_text = ">>> CAN GO DOWN STAIR <<<"
                    alarm_size = cv2.getTextSize(alarm_text, font, 0.8, 2)[0]
                    text_x = (width - alarm_size[0]) // 2
                    cv2.putText(depth_display, alarm_text,
                               (text_x, 80), font, 0.8, (0, 0, 255), 3)

                cv2.putText(depth_display, f"FPS: {fps_display:.1f}",
                           (width - 80, 30), font, 0.5, (255, 255, 255), 1)

                cv2.imshow(self.window_name, depth_display)

                if frame_count % 30 == 0:
                    status = "ABOVE" if valid_ratio >= self.threshold_ratio else "BELOW"
                    print(f"Frame {frame_count} ({fps_display:.1f} FPS): "
                          f"Distance = {distance*100 if distance else 0:.2f} cm, "
                          f"Ratio = {valid_ratio*100:.1f}%, "
                          f"Threshold = {self.threshold_ratio*100:.0f}%, "
                          f"Status = {status}")

                ch = cv2.waitKey(1) & 0xFF
                if ch == 27 or ch == ord('q') or ch == ord('Q'):
                    print("\nUser requested exit")
                    break

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
            cv2.destroyAllWindows()
            print("Done.")


def main():
    app = StripDetectorApp()
    app.run()


if __name__ == '__main__':
    main()
