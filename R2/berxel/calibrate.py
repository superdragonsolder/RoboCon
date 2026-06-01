#coding=utf-8
import sys
import os
import time
import threading
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from howlong.berxel.src.camera_manager import CameraManager
import cv2


class CalibrationTool:
    def __init__(self):
        self.roi_size = 10
        self.roi_x = 320
        self.roi_y = 200
        self.consecutive_failures = 0
        self.max_failures = 50
        self.measurements = []
        self.current_median = 0
        self.running = True
        self.waiting_for_input = False
        self.input_value = None
        self.input_result = None
        self.input_lock = threading.Lock()
    
    def input_thread_func(self, count, median_val):
        try:
            result = {}
            result['msg'] = f"\n>>> 第{count}次测量 <<<\n    ROI原始值: {median_val:.0f}\n    请输入实际距离(cm): "
            result['value'] = float(input(result['msg']).strip())
            with self.input_lock:
                self.input_value = result['value']
                self.input_result = result['value']
                self.waiting_for_input = False
        except ValueError:
            with self.input_lock:
                self.input_value = None
                self.input_result = None
                self.waiting_for_input = False
            print("    输入无效!")
        except Exception as e:
            with self.input_lock:
                self.input_value = None
                self.input_result = None
                self.waiting_for_input = False
            print(f"    错误: {e}")
    
    def run(self):
        print("=" * 70)
        print("Berxel 100R 精确校准工具")
        print("=" * 70)
        print("\n使用说明:")
        print("1. 将相机固定在已知高度,用尺子测量实际距离")
        print("2. 点击画面选择ROI中心位置")
        print("3. +/-调整ROI大小")
        print("4. 按Enter键测量,终端会询问实际距离")
        print("5. 改变高度,重复3次")
        print("6. 按q退出")
        print("-" * 70)
        
        camera = CameraManager()
        
        if not camera.initialize():
            print("相机初始化失败")
            return
        
        if not camera.start_depth_stream(denoise=False, with_color=False):
            print("深度流启动失败")
            camera.close()
            return
        
        window_name = "Calibration - Click to set ROI, Enter to measure"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 640, 400)
        
        print("\n相机已就绪!")
        print("调整ROI位置后,按Enter开始第1次测量...")
        
        frame_count = 0
        input_thread = None
        
        try:
            while self.running and len(self.measurements) < 3:
                depth_frame, pixel_type = camera.get_depth_frame(timeout=500)
                
                if depth_frame is None:
                    self.consecutive_failures += 1
                    if self.consecutive_failures >= self.max_failures:
                        print("USB连接不稳定")
                        break
                    time.sleep(0.05)
                    ch = cv2.waitKey(1) & 0xFF
                    if ch == ord('q') or ch == 27:
                        self.running = False
                    continue
                
                self.consecutive_failures = 0
                
                roi_values = self._extract_roi(depth_frame)
                valid = roi_values[roi_values > 0]
                median_value = np.median(valid) if len(valid) > 0 else 0
                self.current_median = median_value
                distance = median_value / 7900.0 * 100 if median_value > 0 else None
                
                display = self._create_display(depth_frame, median_value, distance)
                
                if self.waiting_for_input:
                    cv2.putText(display, "WAITING FOR INPUT...", (150, 200),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                cv2.imshow(window_name, display)
                
                ch = cv2.waitKey(1) & 0xFF
                if ch == ord('q') or ch == 27:
                    self.running = False
                    break
                elif ch == ord('+') or ch == ord('='):
                    self.roi_size = min(50, self.roi_size + 2)
                    print(f"ROI size: {self.roi_size}")
                elif ch == ord('-'):
                    self.roi_size = max(2, self.roi_size - 2)
                    print(f"ROI size: {self.roi_size}")
                elif ch == 13 or ch == 10:
                    if not self.waiting_for_input and median_value > 0:
                        count = len(self.measurements) + 1
                        self.waiting_for_input = True
                        self.input_value = None
                        input_thread = threading.Thread(
                            target=self.input_thread_func,
                            args=(count, median_value),
                            daemon=True
                        )
                        input_thread.start()
                    elif not self.waiting_for_input and median_value == 0:
                        print("无效数据,ROI区域无有效深度值")
                
                if self.input_result is not None:
                    factor = self.current_median / self.input_result
                    self.measurements.append({
                        'median': self.current_median,
                        'actual': self.input_result,
                        'factor': factor
                    })
                    print(f"    转换因子: {factor:.2f}")
                    print(f"    已完成 {len(self.measurements)}/3 次测量")
                    
                    current_avg = np.mean([m['factor'] for m in self.measurements])
                    print(f"    当前平均因子: {current_avg:.2f}")
                    
                    if len(self.measurements) < 3:
                        print(f"\n改变高度后,按Enter进行第{len(self.measurements)+1}次测量...")
                    self.input_result = None
                
                frame_count += 1
        
        except KeyboardInterrupt:
            print("\n中断")
        finally:
            cv2.destroyAllWindows()
            camera.close()
            
            if len(self.measurements) > 0:
                avg_factor = np.mean([m['factor'] for m in self.measurements])
                
                print(f"\n{'='*70}")
                print(f"校准结果 ({len(self.measurements)}次测量)")
                print(f"{'='*70}")
                for i, m in enumerate(self.measurements):
                    print(f"  第{i+1}次: 原始值={m['median']:.0f}, 实际={m['actual']}cm, 因子={m['factor']:.2f}")
                print(f"{'='*70}")
                print(f"平均转换因子: {avg_factor:.2f}")
                print(f"{'='*70}")
                print(f"\n请修改以下文件中的 7900.0 为 {avg_factor:.2f}:")
                print(f"  - src/distance_measurer.py")
                print(f"  - src/visualizer.py")
                print(f"  - src/camera_manager.py")
            else:
                print("未进行任何测量")
            
            print("完成")
    
    def _extract_roi(self, depth_frame):
        height, width = depth_frame.shape
        half_roi = self.roi_size // 2
        x_start = max(0, self.roi_x - half_roi)
        x_end = min(width, self.roi_x + half_roi)
        y_start = max(0, self.roi_y - half_roi)
        y_end = min(height, self.roi_y + half_roi)
        roi = depth_frame[y_start:y_end, x_start:x_end]
        return roi[roi > 0]
    
    def _create_display(self, depth_array, median_value, distance):
        depth_meters = depth_array.astype(np.float32) / 7900.0
        valid_depth = depth_meters[depth_meters > 0]
        if len(valid_depth) > 0:
            min_depth = np.percentile(valid_depth, 5)
            max_depth = np.percentile(valid_depth, 95)
        else:
            min_depth = 0
            max_depth = 10.0
        if max_depth == min_depth:
            max_depth = min_depth + 1.0
        depth_normalized = np.clip((depth_meters - min_depth) / (max_depth - min_depth), 0, 1)
        depth_normalized[depth_meters == 0] = 0
        depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)
        depth_colored[depth_meters == 0] = [0, 0, 0]
        
        height, width = depth_colored.shape[:2]
        half_roi = self.roi_size // 2
        cv2.rectangle(depth_colored,
                     (self.roi_x - half_roi, self.roi_y - half_roi),
                     (self.roi_x + half_roi, self.roi_y + half_roi),
                     (0, 255, 0), 2)
        cv2.circle(depth_colored, (self.roi_x, self.roi_y), 3, (0, 255, 255), -1)
        
        if distance is not None:
            cv2.putText(depth_colored, f"Dist: {distance:.2f} cm", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(depth_colored, f"Raw: {median_value:.0f}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
        cv2.putText(depth_colored, f"ROI: ({self.roi_x},{self.roi_y}) Size: {self.roi_size}",
                   (10, height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        msg = "Enter=measure q=quit"
        cv2.putText(depth_colored, msg, (10, height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        if len(self.measurements) > 0:
            avg_factor = np.mean([m['factor'] for m in self.measurements])
            cv2.putText(depth_colored, f"AVG Factor: {avg_factor:.2f}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return depth_colored


if __name__ == '__main__':
    tool = CalibrationTool()
    tool.run()
