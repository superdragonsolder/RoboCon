#coding=utf-8
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from howlong.berxel.src.camera_manager import CameraManager
from howlong.berxel.src.distance_measurer import DistanceMeasurer
from howlong.berxel.src.visualizer import Visualizer


def main():
    print("=" * 60)
    print("Berxel Depth Camera - Ground Distance Measurement")
    print("=" * 60)

    camera = CameraManager()

    if not camera.initialize():
        print("Failed to initialize camera. Exiting...")
        return

    if not camera.start_depth_stream(denoise=False, with_color=False):
        print("Failed to start depth stream. Exiting...")
        camera.close()
        return

    has_color = camera.has_color_stream()

    measurer = DistanceMeasurer(roi_size=10, temporal_buffer_size=5, enable_temporal_filter=True)

    visualizer = Visualizer(window_name="Ground Distance Measurement")
    visualizer.set_roi_size(10)

    print("\nStarting measurement...")
    print("Press 'q' or 'ESC' to quit")
    print("-" * 60)

    frame_count = 0
    timeout_count = 0
    max_timeout = 100
    last_frame_time = time.time()
    fps_counter = 0
    fps_display = 0

    try:
        while True:
            depth_frame, pixel_type = camera.get_depth_frame(timeout=1000)

            if depth_frame is None:
                timeout_count += 1
                if timeout_count >= max_timeout:
                    print(f"\nUSB connection unstable ({timeout_count} timeouts)")
                    print("Please check USB cable or use powered USB hub")
                    break
                continue
            
            timeout_count = 0
            frame_count += 1
            fps_counter += 1

            current_time = time.time()
            elapsed = current_time - last_frame_time
            if elapsed >= 1.0:
                fps_display = fps_counter / elapsed
                fps_counter = 0
                last_frame_time = current_time

            distance, valid_count, total_count = measurer.measure_with_temporal_filter(depth_frame, pixel_type)

            stats = measurer.get_statistics(depth_frame, pixel_type)

            color_frame = None
            if has_color:
                color_frame = camera.get_color_frame(timeout=5)

            if not visualizer.show(depth_frame, distance, stats, pixel_type, color_frame):
                print("\nUser requested exit")
                break

            if frame_count % 30 == 0 and stats is not None:
                print(f"Frame {frame_count} ({fps_display:.1f} FPS): Distance = {distance*100:.2f} cm, "
                      f"Valid: {valid_count}/{total_count}, Std: {stats['std']*100:.2f} cm")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nCleaning up...")
        visualizer.cleanup()
        camera.close()
        print("Done.")


if __name__ == '__main__':
    main()
