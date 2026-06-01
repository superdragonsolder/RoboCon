#coding=utf-8
import csv
import os
from datetime import datetime


class DataRecorder:
    def __init__(self, output_dir="data", filename_prefix="measurement"):
        self.output_dir = output_dir
        self.filename_prefix = filename_prefix
        self.csv_file = None
        self.csv_writer = None
        self.is_recording = False

        os.makedirs(output_dir, exist_ok=True)

    def start_recording(self, filename=None):
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.filename_prefix}_{timestamp}.csv"

        filepath = os.path.join(self.output_dir, filename)

        self.csv_file = open(filepath, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'timestamp',
            'frame_index',
            'distance_m',
            'distance_mm',
            'mean_m',
            'median_m',
            'std_m',
            'min_m',
            'max_m',
            'valid_count',
            'total_count',
            'valid_ratio'
        ])

        self.csv_file.flush()
        self.is_recording = True
        print(f"Recording started: {filepath}")

        return filepath

    def record_measurement(self, frame_index, distance, stats):
        if not self.is_recording or self.csv_writer is None:
            return

        timestamp = datetime.now().isoformat()

        if distance is not None:
            distance_m = f"{distance:.6f}"
            distance_mm = f"{distance * 1000:.4f}"
        else:
            distance_m = "N/A"
            distance_mm = "N/A"

        if stats is not None:
            row = [
                timestamp,
                frame_index,
                distance_m,
                distance_mm,
                f"{stats['mean']:.6f}",
                f"{stats['median']:.6f}",
                f"{stats['std']:.6f}",
                f"{stats['min']:.6f}",
                f"{stats['max']:.6f}",
                stats['valid_count'],
                stats['total_count'],
                f"{stats['valid_ratio']:.4f}"
            ]
        else:
            row = [
                timestamp,
                frame_index,
                distance_m,
                distance_mm,
                "N/A",
                "N/A",
                "N/A",
                "N/A",
                "N/A",
                0,
                0,
                0.0
            ]

        self.csv_writer.writerow(row)

    def stop_recording(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        self.is_recording = False
        print("Recording stopped")

    def __del__(self):
        if self.is_recording:
            self.stop_recording()
