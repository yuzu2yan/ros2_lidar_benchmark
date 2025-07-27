#!/usr/bin/env python3

import matplotlib
matplotlib.use('TkAgg')  # Set backend before importing pyplot
import matplotlib.pyplot as plt
import numpy as np
import time

def test_simple_plot():
    """Test basic matplotlib functionality"""
    print("Testing simple matplotlib plot...")
    
    # Create figure
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # Generate data
    x = np.linspace(0, 10, 100)
    y = np.sin(x)
    
    # Plot
    ax.plot(x, y, 'b-', linewidth=2)
    ax.set_title('Test Plot - Sin Wave')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True)
    
    print("Showing plot with plt.show()...")
    plt.show()
    
    print("If you see the plot, matplotlib is working correctly!")

def test_animated_plot():
    """Test animated plot"""
    print("\nTesting animated plot...")
    
    # Enable interactive mode
    plt.ion()
    
    # Create figure
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_xlim(0, 10)
    ax.set_ylim(-2, 2)
    ax.set_title('Animated Test Plot')
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    ax.grid(True)
    
    # Initialize line
    line, = ax.plot([], [], 'r-', linewidth=2)
    
    # Data storage
    xdata = []
    ydata = []
    
    print("Starting animation (press Ctrl+C to stop)...")
    
    start_time = time.time()
    
    try:
        while True:
            current_time = time.time() - start_time
            
            # Add new data
            xdata.append(current_time)
            ydata.append(np.sin(current_time))
            
            # Keep last 100 points
            if len(xdata) > 100:
                xdata = xdata[-100:]
                ydata = ydata[-100:]
            
            # Update line
            line.set_data(xdata, ydata)
            
            # Update axes limits
            if len(xdata) > 0:
                ax.set_xlim(max(0, xdata[-1] - 10), xdata[-1] + 1)
            
            # Draw
            fig.canvas.draw()
            fig.canvas.flush_events()
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nAnimation stopped")
    
    plt.ioff()
    plt.show()

def main():
    print("Matplotlib Visualization Test")
    print("=" * 40)
    
    # Test 1: Simple plot
    test_simple_plot()
    
    # Test 2: Animated plot
    test_animated_plot()

if __name__ == '__main__':
    main()