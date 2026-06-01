#coding=utf-8
import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from howlong.berxel.src.camera_manager import CameraManager
from howlong.berxel.src.distance_measurer import DistanceMeasurer
import cv2


class DistanceDetector:
    def __init__(self, conversion_factor=7900.0, roi_size=10):
        self.conversion_factor = conversion_factor
        self.roi_size = roi_size
        self.measurer = DistanceMeasurer(
            roi_size=roi_size,
            temporal_buffer_size=5,
            enable_temporal_filter=True,
            conversion_factor=conversion_factor
        )
    
    def run(self):
        print("=" * 70)
        print("Distance Detector - Color View with Distance Measurement")
        print("=" * 70)
        print(f"Conversion factor: {self.conversion_factor}")
        print(f"ROI size: {self.roi_size}")
        print("-" * 70)
        print("Press 'q' or 'ESC' to quit")
        print("-" * 70)
        
        camera = CameraManager()
        
        if not camera.initialize():
            print("Failed to initialize camera")
            return
        
        if not camera.start_depth_stream(denoise=False, with_color=True):
            print("Failed to start streams")
            camera.close()
            return
        
        window_name = "Distance Detector"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        print("\nCamera ready!")
        
        consecutive_failures = 0
        max_failures = 50
        
        try:
            while True:
                depth_frame, pixel_type = camera.get_depth_frame(timeout=30)
                color_frame = camera.get_color_frame(timeout=30)
                
                if depth_frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= max_failures:
                        print("USB connection unstable")
                        break
                    time.sleep(0.05)
                    ch = cv2.waitKey(1) & 0xFF
                    if ch == ord('q') or ch == 27:
                        break
                    continue
                
                consecutive_failures = 0
                
                distance, valid_count, total_count = self.measurer.measure_center_distance(depth_frame, pixel_type)
                
                display = color_frame.copy() if color_frame is not None else np.zeros((480, 640, 3), dtype=np.uint8)
                
                height, width = display.shape[:2]
                center_x, center_y = width // 2, height // 2
                half_roi = self.roi_size // 2
                
                cv2.rectangle(display,
                            (center_x - half_roi, center_y - half_roi),
                            (center_x + half_roi, center_y + half_roi),
                            (0, 255, 0), 2)
                cv2.circle(display, (center_x, center_y), 3, (0, 255, 255), -1)
                
                if distance is not None:
                    distance_cm = distance * 100
                    cv2.putText(display, f"Distance: {distance_cm:.1f} cm", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(display, f"Valid pixels: {valid_count}/{total_count}", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                else:
                    cv2.putText(display, "No valid depth data", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                cv2.imshow(window_name, display)
                
                ch = cv2.waitKey(1) & 0xFF
                if ch == ord('q') or ch == 27:
                    break
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"\nError: {e}")
            import traceback
            traceback.print_exc()
        finally:
            cv2.destroyAllWindows()
            camera.close()
            print("Done.")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Distance Detector with Color View')
    parser.add_argument('--conversion-factor', type=float, default=7900.0,
                       help='Depth pixel to distance conversion factor (default: 7900.0)')
    parser.add_argument('--roi-size', type=int, default=10,
                       help='ROI size for distance measurement (default: 10)')
    
    args = parser.parse_args()
    
    detector = DistanceDetector(
        conversion_factor=args.conversion_factor,
        roi_size=args.roi_size
    )
    
    detector.run()


if __name__ == '__main__':
    main()
