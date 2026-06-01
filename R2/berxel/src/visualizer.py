#coding=utf-8
import cv2
import numpy as np


class Visualizer:
    def __init__(self, window_name="Ground Distance Measurement"):
        self.window_name = window_name
        self.distance_history = []
        self.max_history = 100
        self.roi_size = 10

    def set_roi_size(self, roi_size):
        self.roi_size = roi_size

    def create_depth_display(self, depth_array, pixel_type, conversion_factor=79.0):
        depth_meters = depth_array.astype(np.float32) / conversion_factor
        
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

        return depth_colored

    def draw_roi(self, display_image, roi_size):
        height, width = display_image.shape[:2]
        center_y, center_x = height // 2, width // 2
        half_roi = roi_size // 2

        cv2.rectangle(
            display_image,
            (center_x - half_roi, center_y - half_roi),
            (center_x + half_roi, center_y + half_roi),
            (0, 255, 0),
            2
        )

        cv2.circle(display_image, (center_x, center_y), 3, (0, 255, 255), -1)

        return display_image

    def draw_strip_roi(self, display_image, strip_width, strip_height, vertical=False, 
                       y_position=None, valid_ratio=None, distance=None):
        height, width = display_image.shape[:2]
        
        if vertical:
            strip_w = strip_height
            strip_h = strip_width
            center_x = width // 2
            x_start = max(0, center_x - strip_w // 2)
            x_end = min(width, center_x + strip_w // 2)
            
            if y_position is None:
                center_y = height // 2
            else:
                center_y = y_position
            y_start = max(0, center_y - strip_h // 2)
            y_end = min(height, center_y + strip_h // 2)
        else:
            strip_w = strip_width
            strip_h = strip_height
            center_x = width // 2
            x_start = max(0, center_x - strip_w // 2)
            x_end = min(width, center_x + strip_w // 2)
            
            if y_position is None:
                center_y = height // 2
            else:
                center_y = y_position
            y_start = max(0, center_y - strip_h // 2)
            y_end = min(height, center_y + strip_h // 2)

        cv2.rectangle(
            display_image,
            (x_start, y_start),
            (x_end, y_end),
            (0, 255, 0),
            2
        )

        if valid_ratio is not None:
            ratio_text = f"{valid_ratio*100:.1f}%"
            text_x = x_start + 5
            text_y = y_start + 15
            cv2.putText(display_image, ratio_text, (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if distance is not None:
            dist_text = f"{distance*100:.2f}cm"
            text_x = x_start + 5
            text_y = y_end - 5
            cv2.putText(display_image, dist_text, (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return display_image

    def draw_ratio_bar(self, display_image, valid_ratio, threshold_ratio=0.5, 
                       bar_width=200, bar_height=20, x_offset=10, y_offset=200):
        panel_color = (40, 40, 40)
        overlay = display_image.copy()
        panel_x1 = x_offset
        panel_x2 = x_offset + bar_width + 80
        panel_y1 = y_offset - 30
        panel_y2 = y_offset + bar_height + 30
        cv2.rectangle(overlay, (panel_x1, panel_y1), (panel_x2, panel_y2), panel_color, -1)
        cv2.addWeighted(overlay, 0.7, display_image, 0.3, 0, display_image)

        cv2.putText(display_image, f"Ratio: {valid_ratio*100:.1f}%", 
                   (x_offset, y_offset - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        bg_x1 = x_offset
        bg_y1 = y_offset
        bg_x2 = x_offset + bar_width
        bg_y2 = y_offset + bar_height
        cv2.rectangle(display_image, (bg_x1, bg_y1), (bg_x2, bg_y2), (80, 80, 80), -1)

        fill_width = int(bar_width * valid_ratio)
        if fill_width > 0:
            fill_color = (0, 255, 0) if valid_ratio >= threshold_ratio else (0, 128, 255)
            cv2.rectangle(display_image, (bg_x1, bg_y1), 
                         (bg_x1 + fill_width, bg_y2), fill_color, -1)

        threshold_x = int(x_offset + bar_width * threshold_ratio)
        cv2.line(display_image, (threshold_x, bg_y1 - 5), 
                (threshold_x, bg_y2 + 5), (0, 0, 255), 2)

        cv2.putText(display_image, f"Threshold: {threshold_ratio*100:.0f}%", 
                   (x_offset, y_offset + bar_height + 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        return display_image

    def draw_info_panel(self, display_image, distance, stats, pixel_type):
        height, width = display_image.shape[:2]

        panel_height = 180
        panel_color = (40, 40, 40)
        text_color = (255, 255, 255)
        highlight_color = (0, 255, 255)

        overlay = display_image.copy()
        cv2.rectangle(overlay, (0, 0), (width, panel_height), panel_color, -1)
        cv2.addWeighted(overlay, 0.7, display_image, 0.3, 0, display_image)

        y_pos = 25
        font = cv2.FONT_HERSHEY_SIMPLEX

        pixel_type_str = "12I_4D" if pixel_type == 0x01 else "13I_3D" if pixel_type == 0x02 else "Unknown"
        cv2.putText(display_image, f"Pixel Type: {pixel_type_str}", 
                   (10, y_pos), font, 0.5, text_color, 1)
        y_pos += 25

        if distance is not None:
            distance_cm = distance * 100
            cv2.putText(display_image, f"Distance: {distance_cm:.2f} cm", 
                       (10, y_pos), font, 0.7, highlight_color, 2)
            y_pos += 25
            cv2.putText(display_image, f"Distance: {distance:.4f} m", 
                       (10, y_pos), font, 0.6, text_color, 1)
            y_pos += 30
        else:
            cv2.putText(display_image, "Distance: N/A (no valid data)", 
                       (10, y_pos), font, 0.6, (0, 0, 255), 2)
            y_pos += 30

        if stats is not None:
            cv2.putText(display_image, f"Mean: {stats['mean']*100:.2f} cm  Std: {stats['std']*100:.2f} cm",
                       (10, y_pos), font, 0.5, text_color, 1)
            y_pos += 20
            cv2.putText(display_image, f"Min: {stats['min']*100:.2f} cm  Max: {stats['max']*100:.2f} cm",
                       (10, y_pos), font, 0.5, text_color, 1)
            y_pos += 20
            cv2.putText(display_image, f"Valid: {stats['valid_count']}/{stats['total_count']} ({stats['valid_ratio']*100:.1f}%)",
                       (10, y_pos), font, 0.5, text_color, 1)

        return display_image

    def show(self, depth_array, distance, stats, pixel_type, color_frame=None):
        conversion_factor = 7900.0 if pixel_type == 0x02 else 16.0

        depth_display = self.create_depth_display(depth_array, pixel_type, conversion_factor)

        depth_display = self.draw_roi(depth_display, self.roi_size)

        depth_display = self.draw_info_panel(depth_display, distance, stats, pixel_type)

        if color_frame is not None:
            color_h, color_w = color_frame.shape[:2]
            depth_h, depth_w = depth_display.shape[:2]

            target_h = max(color_h, depth_h)

            color_scale = target_h / color_h
            depth_scale = target_h / depth_h

            color_resized = cv2.resize(color_frame, (int(color_w * color_scale), int(color_h * color_scale)))
            depth_resized = cv2.resize(depth_display, (int(depth_w * depth_scale), int(depth_h * depth_scale)))

            cv2.putText(color_resized, "Color", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(depth_resized, "Depth", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            combined = np.hstack((color_resized, depth_resized))
        else:
            combined = depth_display

        if distance is not None:
            self.distance_history.append(distance)
            if len(self.distance_history) > self.max_history:
                self.distance_history.pop(0)

        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self.window_name, combined)

        ch = cv2.waitKey(1) & 0xFF

        if ch == 27 or ch == ord('q') or ch == ord('Q'):
            return False

        return True

    @staticmethod
    def cleanup():
        cv2.destroyAllWindows()
