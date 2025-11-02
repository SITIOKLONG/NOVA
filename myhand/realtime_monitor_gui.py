#!/usr/bin/env python
"""
Real-Time Joint Monitoring GUI
PyQt5 + Matplotlib GUI for monitoring servo safety data in real-time
"""

import sys
import time
from datetime import datetime
from collections import defaultdict
import numpy as np

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

from servo_driver import ServoDriver


class MonitorSignals(QObject):
    """Signals for thread-safe GUI updates"""
    data_update = pyqtSignal(str, dict)  # joint_name, data_point


class RealtimePlot(FigureCanvas):
    """Matplotlib canvas for real-time plotting"""
    
    def __init__(self, parent=None, width=12, height=9):
        self.fig = Figure(figsize=(width, height), dpi=100)
        super().__init__(self.fig)
        self.setParent(parent)
        
        # Create subplots
        self.ax_load = self.fig.add_subplot(311)
        self.ax_position = self.fig.add_subplot(312)
        self.ax_metrics = self.fig.add_subplot(313)
        
        self.fig.tight_layout(pad=3.0)
        
        # Data storage
        self.data = defaultdict(lambda: {
            'timestamps': [],
            'loads': [],
            'baseline_loads': [],
            'thresholds': [],
            'positions': [],
            'load_increases': [],
            'distances': []
        })
        
        # Plot lines
        self.lines = {}
        
        # Setup plots
        self._setup_plots()
    
    def _setup_plots(self):
        """Initialize plot appearance"""
        # Load plot
        self.ax_load.set_xlabel('Time (s)', fontsize=10)
        self.ax_load.set_ylabel('Load / Torque', fontsize=10)
        self.ax_load.set_title('Load vs Torque Threshold', fontsize=12, fontweight='bold')
        self.ax_load.grid(True, alpha=0.3)
        self.ax_load.legend(loc='upper right', fontsize=9)
        
        # Position plot
        self.ax_position.set_xlabel('Time (s)', fontsize=10)
        self.ax_position.set_ylabel('Position', fontsize=10)
        self.ax_position.set_title('Servo Position', fontsize=12, fontweight='bold')
        self.ax_position.grid(True, alpha=0.3)
        self.ax_position.legend(loc='upper right', fontsize=9)
        
        # Metrics plot
        self.ax_metrics.set_xlabel('Time (s)', fontsize=10)
        self.ax_metrics.set_ylabel('Load Increase / Distance', fontsize=10)
        self.ax_metrics.set_title('Load Increase & Distance to Target', fontsize=12, fontweight='bold')
        self.ax_metrics.grid(True, alpha=0.3)
        self.ax_metrics.legend(loc='upper right', fontsize=9)
    
    def add_joint(self, joint_name, color):
        """Add a new joint to track"""
        if joint_name in self.lines:
            return
        
        # Create lines for this joint
        line_load, = self.ax_load.plot([], [], '-', linewidth=2, 
                                        label=f'{joint_name} Load', color=color)
        line_baseline, = self.ax_load.plot([], [], '--', linewidth=1, 
                                            label=f'{joint_name} Baseline', 
                                            color=color, alpha=0.5)
        line_threshold, = self.ax_load.plot([], [], ':', linewidth=2, 
                                             label=f'{joint_name} Threshold', 
                                             color='red', alpha=0.7)
        
        line_pos, = self.ax_position.plot([], [], '-', linewidth=2,
                                          label=joint_name, color=color)
        
        line_load_inc, = self.ax_metrics.plot([], [], '-', linewidth=2,
                                              label=f'{joint_name} Load Inc', color=color)
        line_dist, = self.ax_metrics.plot([], [], '--', linewidth=1,
                                          label=f'{joint_name} Distance', 
                                          color=color, alpha=0.5)
        
        self.lines[joint_name] = {
            'load': line_load,
            'baseline': line_baseline,
            'threshold': line_threshold,
            'position': line_pos,
            'load_increase': line_load_inc,
            'distance': line_dist
        }
        
        # Update legends
        self.ax_load.legend(loc='upper right', fontsize=8)
        self.ax_position.legend(loc='upper right', fontsize=8)
        self.ax_metrics.legend(loc='upper right', fontsize=8)
    
    def update_data(self, joint_name, data_point):
        """Update data for a joint"""
        d = self.data[joint_name]
        d['timestamps'].append(data_point['timestamp'])
        d['loads'].append(data_point['load'])
        d['baseline_loads'].append(data_point['baseline_load'])
        d['thresholds'].append(data_point['torque_threshold'])
        d['positions'].append(data_point['position'])
        d['load_increases'].append(data_point['load_increase'])
        d['distances'].append(data_point['distance_to_target'])
        
        # Keep only last 500 points for performance
        max_points = 500
        for key in d:
            if len(d[key]) > max_points:
                d[key] = d[key][-max_points:]
    
    def refresh_plot(self):
        """Refresh all plots with current data"""
        for joint_name, d in self.data.items():
            if joint_name not in self.lines or not d['timestamps']:
                continue
            
            lines = self.lines[joint_name]
            
            # Update load plot
            lines['load'].set_data(d['timestamps'], d['loads'])
            lines['baseline'].set_data(d['timestamps'], d['baseline_loads'])
            lines['threshold'].set_data(d['timestamps'], d['thresholds'])
            
            # Update position plot
            lines['position'].set_data(d['timestamps'], d['positions'])
            
            # Update metrics plot
            lines['load_increase'].set_data(d['timestamps'], d['load_increases'])
            lines['distance'].set_data(d['timestamps'], d['distances'])
        
        # Rescale axes
        for ax in [self.ax_load, self.ax_position, self.ax_metrics]:
            ax.relim()
            ax.autoscale_view()
        
        self.draw()
    
    def clear_all(self):
        """Clear all data"""
        self.data.clear()
        for lines_dict in self.lines.values():
            for line in lines_dict.values():
                line.remove()
        self.lines.clear()
        self.ax_load.legend(loc='upper right', fontsize=8)
        self.ax_position.legend(loc='upper right', fontsize=8)
        self.ax_metrics.legend(loc='upper right', fontsize=8)
        self.draw()


class MonitorGUI(QMainWindow):
    """Main GUI window for real-time monitoring"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ORCA Hand Real-Time Monitor")
        self.setGeometry(100, 100, 600, 600)
        
        # Data
        self.signals = MonitorSignals()
        self.joint_colors = {}
        
        # Setup GUI
        self.setup_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.on_timer_update)
        self.update_timer.start(10)  # Update every 50ms
        
        # Connect signals
        self.signals.data_update.connect(self.on_data_update)
    
    def setup_ui(self):
        """Setup the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Real-time plot
        self.canvas = RealtimePlot(central_widget, width=14, height=10)
        main_layout.addWidget(self.canvas)
        
        # Status bar
        self.statusBar().showMessage("Real-Time Monitoring Active")
    
    def register_joint(self, joint_name, color=None):
        """Register a joint for monitoring"""
        if joint_name not in self.joint_colors:
            if color is None:
                # Auto-assign color
                n = len(self.joint_colors)
                colors = plt.cm.tab20(np.linspace(0, 1, 20))
                color = colors[n % 20]
            self.joint_colors[joint_name] = color
            self.canvas.add_joint(joint_name, color)
    
    def update_data(self, joint_name, data_point):
        """Update data for a joint (thread-safe)"""
        self.signals.data_update.emit(joint_name, data_point)
    
    def on_data_update(self, joint_name, data_point):
        """Handle real-time data update"""
        if joint_name not in self.canvas.lines:
            self.register_joint(joint_name)
        self.canvas.update_data(joint_name, data_point)
    
    def on_timer_update(self):
        """Periodic update of plots"""
        if self.canvas.data:
            self.canvas.refresh_plot()
    
    def clear_plots(self):
        """Clear all plot data"""
        self.canvas.clear_all()


def main():
    app = QApplication(sys.argv)
    window = MonitorGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
