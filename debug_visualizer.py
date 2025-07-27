#!/usr/bin/env python3

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import time
from collections import deque

def test_basic_plot():
    """Test if matplotlib works at all"""
    print("Testing basic matplotlib...")
    
    fig, ax = plt.subplots()
    x = np.linspace(0, 10, 100)
    y = np.sin(x)
    ax.plot(x, y)
    ax.set_title('Test Plot')
    
    plt.show()
    print("If you saw a sine wave, matplotlib is working.")

def test_realtime_update():
    """Test real-time updating"""
    print("\nTesting real-time updates...")
    
    plt.ion()  # Interactive mode
    fig, ax = plt.subplots()
    
    # Initialize empty line
    line, = ax.plot([], [], 'b-')
    ax.set_xlim(0, 10)
    ax.set_ylim(-2, 2)
    ax.set_title('Real-time Update Test')
    ax.grid(True)
    
    xdata = deque(maxlen=100)
    ydata = deque(maxlen=100)
    
    start_time = time.time()
    
    print("Starting real-time update (press Ctrl+C to stop)...")
    
    try:
        while True:
            current_time = time.time() - start_time
            
            # Add new data
            xdata.append(current_time)
            ydata.append(np.sin(current_time * 2))
            
            # Update line data
            line.set_xdata(list(xdata))
            line.set_ydata(list(ydata))
            
            # Update x-axis limits
            if len(xdata) > 0:
                ax.set_xlim(max(0, xdata[-1] - 10), xdata[-1] + 1)
            
            # Redraw
            ax.figure.canvas.draw()
            ax.figure.canvas.flush_events()
            
            time.sleep(0.05)  # 20 Hz update
            
    except KeyboardInterrupt:
        print("\nStopped.")
    
    plt.ioff()
    plt.show()

def test_multiple_subplots():
    """Test multiple subplots like the visualizer"""
    print("\nTesting multiple subplots...")
    
    plt.ion()
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 8))
    fig.suptitle('Multiple Subplot Test')
    
    # Setup axes
    for ax, title in zip([ax1, ax2, ax3, ax4], 
                        ['Plot 1', 'Plot 2', 'Plot 3', 'Plot 4']):
        ax.set_title(title)
        ax.set_xlim(0, 10)
        ax.set_ylim(-2, 2)
        ax.grid(True)
    
    # Create lines
    lines = []
    for ax in [ax1, ax2, ax3, ax4]:
        line, = ax.plot([], [], 'r-')
        lines.append(line)
    
    # Data storage
    data_queues = [deque(maxlen=100) for _ in range(4)]
    time_queue = deque(maxlen=100)
    
    start_time = time.time()
    
    print("Updating multiple plots (press Ctrl+C to stop)...")
    
    try:
        while True:
            current_time = time.time() - start_time
            time_queue.append(current_time)
            
            # Generate different data for each plot
            data_queues[0].append(np.sin(current_time))
            data_queues[1].append(np.cos(current_time))
            data_queues[2].append(np.sin(current_time * 2))
            data_queues[3].append(np.cos(current_time * 2))
            
            # Update all lines
            for line, data in zip(lines, data_queues):
                line.set_xdata(list(time_queue))
                line.set_ydata(list(data))
            
            # Update x-axis limits
            if len(time_queue) > 0:
                for ax in [ax1, ax2, ax3, ax4]:
                    ax.set_xlim(max(0, time_queue[-1] - 10), time_queue[-1] + 1)
            
            # Redraw
            fig.canvas.draw()
            fig.canvas.flush_events()
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopped.")
    
    plt.ioff()
    plt.show()

def main():
    print("Matplotlib Debugging Tool")
    print("========================")
    
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == '1':
            test_basic_plot()
        elif sys.argv[1] == '2':
            test_realtime_update()
        elif sys.argv[1] == '3':
            test_multiple_subplots()
    else:
        print("\nUsage:")
        print("  python3 debug_visualizer.py 1  # Test basic plot")
        print("  python3 debug_visualizer.py 2  # Test real-time update")
        print("  python3 debug_visualizer.py 3  # Test multiple subplots")
        print("\nRunning all tests...")
        
        test_basic_plot()
        input("\nPress Enter to continue to real-time test...")
        test_realtime_update()
        input("\nPress Enter to continue to multiple subplot test...")
        test_multiple_subplots()

if __name__ == '__main__':
    main()