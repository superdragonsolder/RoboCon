#coding=utf-8
import numpy as np
from collections import deque


class DistanceMeasurer:
    PIXEL_TYPE_12I_4D = 0x01
    PIXEL_TYPE_13I_3D = 0x02

    def __init__(self, roi_size=10, temporal_buffer_size=10, enable_temporal_filter=True, conversion_factor=7900.0):
        self.roi_size = roi_size
        self.enable_temporal_filter = enable_temporal_filter
        self.temporal_buffer = deque(maxlen=temporal_buffer_size)
        self.conversion_factor = conversion_factor

    def set_conversion_factor(self, factor):
        self.conversion_factor = factor

    def convert_depth_to_distance(self, depth_array, pixel_type):
        if pixel_type == self.PIXEL_TYPE_12I_4D:
            return depth_array.astype(np.float32) / 16.0
        elif pixel_type == self.PIXEL_TYPE_13I_3D:
            return depth_array.astype(np.float32) / self.conversion_factor
        else:
            return depth_array.astype(np.float32) / self.conversion_factor

    def measure_center_distance(self, depth_array, pixel_type):
        height, width = depth_array.shape
        center_y, center_x = height // 2, width // 2
        half_roi = self.roi_size // 2

        y_start = max(0, center_y - half_roi)
        y_end = min(height, center_y + half_roi)
        x_start = max(0, center_x - half_roi)
        x_end = min(width, center_x + half_roi)

        roi = depth_array[y_start:y_end, x_start:x_end]

        distance_array = self.convert_depth_to_distance(roi, pixel_type)

        valid_distances = distance_array[distance_array > 0]

        if len(valid_distances) == 0:
            return None, 0, len(distance_array)

        median_distance = float(np.median(valid_distances))
        return median_distance, len(valid_distances), len(distance_array)

    def measure_strip_distance(self, depth_array, pixel_type, strip_width=100, strip_height=30, 
                                vertical=False, y_position=None):
        height, width = depth_array.shape
        
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

        roi = depth_array[y_start:y_end, x_start:x_end]

        distance_array = self.convert_depth_to_distance(roi, pixel_type)

        valid_distances = distance_array[distance_array > 0]
        total_pixels = distance_array.size
        valid_count = len(valid_distances)
        valid_ratio = valid_count / total_pixels if total_pixels > 0 else 0.0

        if valid_count == 0:
            return None, valid_count, total_pixels, valid_ratio

        median_distance = float(np.median(valid_distances))
        return median_distance, valid_count, total_pixels, valid_ratio

    def measure_with_temporal_filter(self, depth_array, pixel_type):
        current_distance, valid_count, total_count = self.measure_center_distance(depth_array, pixel_type)

        if current_distance is None:
            if len(self.temporal_buffer) > 0:
                return float(np.mean(self.temporal_buffer)), valid_count, total_count
            return None, valid_count, total_count

        if self.enable_temporal_filter:
            self.temporal_buffer.append(current_distance)
            filtered_distance = float(np.mean(self.temporal_buffer))
            return filtered_distance, valid_count, total_count
        else:
            return current_distance, valid_count, total_count

    def get_statistics(self, depth_array, pixel_type):
        distance_array = self.convert_depth_to_distance(depth_array, pixel_type)
        valid_distances = distance_array[distance_array > 0]

        if len(valid_distances) == 0:
            return None

        stats = {
            'mean': float(np.mean(valid_distances)),
            'median': float(np.median(valid_distances)),
            'std': float(np.std(valid_distances)),
            'min': float(np.min(valid_distances)),
            'max': float(np.max(valid_distances)),
            'valid_count': int(len(valid_distances)),
            'total_count': int(distance_array.size),
            'valid_ratio': float(len(valid_distances) / distance_array.size),
        }

        return stats

    def get_full_frame_statistics(self, depth_array, pixel_type):
        distance_array = self.convert_depth_to_distance(depth_array, pixel_type)
        valid_distances = distance_array[distance_array > 0]

        if len(valid_distances) == 0:
            return None

        stats = {
            'mean': float(np.mean(valid_distances)),
            'median': float(np.median(valid_distances)),
            'std': float(np.std(valid_distances)),
            'min': float(np.min(valid_distances)),
            'max': float(np.max(valid_distances)),
            'percentile_5': float(np.percentile(valid_distances, 5)),
            'percentile_25': float(np.percentile(valid_distances, 25)),
            'percentile_75': float(np.percentile(valid_distances, 75)),
            'percentile_95': float(np.percentile(valid_distances, 95)),
            'valid_count': int(len(valid_distances)),
            'total_count': int(distance_array.size),
        }

        return stats

    def reset_temporal_buffer(self):
        self.temporal_buffer.clear()

    def set_roi_size(self, roi_size):
        if roi_size > 0 and roi_size % 2 == 0:
            self.roi_size = roi_size
        elif roi_size > 0:
            self.roi_size = roi_size if roi_size % 2 == 1 else roi_size + 1
        else:
            print("Warning: ROI size must be positive")

    def set_temporal_buffer_size(self, buffer_size):
        if buffer_size > 0:
            self.temporal_buffer = deque(maxlen=buffer_size)
        else:
            print("Warning: Buffer size must be positive")

    def analyze_height_distribution(self, depth_array, pixel_type, strip_width=100, strip_height=30,
                                    vertical=False, y_position=None, tolerance_percent=5.0):
        height, width = depth_array.shape
        
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

        roi = depth_array[y_start:y_end, x_start:x_end]

        distance_array = self.convert_depth_to_distance(roi, pixel_type)

        valid_distances = distance_array[distance_array > 0]
        total_pixels = distance_array.size
        valid_count = len(valid_distances)
        valid_ratio = valid_count / total_pixels if total_pixels > 0 else 0.0

        if valid_count == 0:
            return {
                'distance': None,
                'valid_count': 0,
                'total_count': total_pixels,
                'valid_ratio': 0.0,
                'height_levels': [],
                'level_percentages': [],
                'num_different_heights': 0,
                'is_uniform': True
            }

        median_distance = float(np.median(valid_distances))
        
        sorted_distances = np.sort(valid_distances)
        
        tolerance_meters = tolerance_percent / 100.0 * median_distance
        
        height_levels = []
        level_counts = []
        
        if len(sorted_distances) > 0:
            current_level = [sorted_distances[0]]
            
            for i in range(1, len(sorted_distances)):
                # Compare with the mean of current level, not the first element
                current_mean = np.mean(current_level)
                if abs(sorted_distances[i] - current_mean) <= tolerance_meters:
                    current_level.append(sorted_distances[i])
                else:
                    if current_level:
                        height_levels.append(float(np.mean(current_level)))
                        level_counts.append(len(current_level))
                    current_level = [sorted_distances[i]]
            
            if current_level:
                height_levels.append(float(np.mean(current_level)))
                level_counts.append(len(current_level))
        
        level_percentages = [(count / valid_count * 100) for count in level_counts]
        
        # Sort by height level
        combined = list(zip(height_levels, level_percentages))
        combined.sort()  # Sort by the first element (height level)
        height_levels_sorted, level_percentages_sorted = zip(*combined) if combined else ([], [])
        
        height_levels_cm = [h * 100 for h in height_levels_sorted]
        
        is_uniform = len(height_levels) <= 1
        
        return {
            'distance': median_distance,
            'valid_count': valid_count,
            'total_count': total_pixels,
            'valid_ratio': valid_ratio,
            'height_levels': height_levels_cm,
            'level_percentages': list(level_percentages_sorted),
            'num_different_heights': len(height_levels),
            'is_uniform': is_uniform
        }
