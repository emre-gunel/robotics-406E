#!/usr/bin/env python3
"""
Generate occupancy grid map from Gazebo world file obstacle positions.
"""
import numpy as np
from PIL import Image
import os

# Map parameters
RESOLUTION = 0.05  # meters per pixel
WORLD_WIDTH = 20.0  # meters
WORLD_HEIGHT = 10.0  # meters

# Add padding around the world
PADDING = 1.0  # meters

# Total map dimensions
MAP_WIDTH = WORLD_WIDTH + 2 * PADDING
MAP_HEIGHT = WORLD_HEIGHT + 2 * PADDING

# Pixel dimensions
WIDTH_PX = int(MAP_WIDTH / RESOLUTION)
HEIGHT_PX = int(MAP_HEIGHT / RESOLUTION)

# Origin (bottom-left corner in world coordinates)
ORIGIN_X = -(WORLD_WIDTH / 2 + PADDING)
ORIGIN_Y = -(WORLD_HEIGHT / 2 + PADDING)

def world_to_pixel(x, y):
    """Convert world coordinates to pixel coordinates."""
    px = int((x - ORIGIN_X) / RESOLUTION)
    py = int((y - ORIGIN_Y) / RESOLUTION)
    return px, py

def draw_circle(grid, cx, cy, radius, value=0):
    """Draw a filled circle on the grid."""
    px, py = world_to_pixel(cx, cy)
    r_px = int(radius / RESOLUTION) + 2  # Add buffer
    
    for i in range(-r_px, r_px + 1):
        for j in range(-r_px, r_px + 1):
            if i*i + j*j <= r_px*r_px:
                x, y = px + i, py + j
                if 0 <= x < WIDTH_PX and 0 <= y < HEIGHT_PX:
                    grid[y, x] = value

def draw_box(grid, cx, cy, width, height, rotation=0, value=0):
    """Draw a filled box on the grid."""
    import math
    
    # Handle rotation (swap width/height for 90 degree rotation)
    if abs(rotation - 1.57) < 0.1:  # ~90 degrees
        width, height = height, width
    
    half_w = width / 2
    half_h = height / 2
    
    # Draw the box
    for dx in np.arange(-half_w, half_w, RESOLUTION):
        for dy in np.arange(-half_h, half_h, RESOLUTION):
            wx = cx + dx
            wy = cy + dy
            px, py = world_to_pixel(wx, wy)
            if 0 <= px < WIDTH_PX and 0 <= py < HEIGHT_PX:
                grid[py, px] = value

def main():
    # Create white (free space) grid
    # PGM: 255 = free, 0 = occupied, 205 = unknown
    grid = np.ones((HEIGHT_PX, WIDTH_PX), dtype=np.uint8) * 255
    
    # Draw boundary walls
    # Wall north: y=5.1, 20m x 0.2m
    draw_box(grid, 0, 5.1, 20.0, 0.4)
    # Wall south: y=-5.1
    draw_box(grid, 0, -5.1, 20.0, 0.4)
    # Wall east: x=10.1
    draw_box(grid, 10.1, 0, 0.4, 10.0)
    # Wall west: x=-10.1
    draw_box(grid, -10.1, 0, 0.4, 10.0)
    
    # Draw trees (cylinders - use circles)
    trees = [
        (-8.0, 4.0, 0.15),   # tree_1
        (-5.0, -3.5, 0.18),  # tree_2
        (3.0, 3.5, 0.12),    # tree_3
        (8.0, -3.0, 0.2),    # tree_4
        (-2.0, 2.0, 0.14),   # tree_5
        (5.5, 1.5, 0.16),    # tree_6
    ]
    
    for x, y, r in trees:
        # Add safety margin around trees
        draw_circle(grid, x, y, r + 0.3)
    
    # Draw benches
    # Bench 1: (7.0, 0) rotated 90deg -> 0.5 x 1.5
    draw_box(grid, 7.0, 0, 0.5, 1.5, rotation=1.57)
    # Bench 2: (-6.0, -1.0) -> 1.5 x 0.5
    draw_box(grid, -6.0, -1.0, 1.5, 0.5)
    
    # Flip vertically (PGM origin is top-left, ROS origin is bottom-left)
    grid = np.flipud(grid)
    
    # Save PGM
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pgm_path = os.path.join(script_dir, 'med_cim_map.pgm')
    yaml_path = os.path.join(script_dir, 'med_cim_map.yaml')
    
    # Save as PGM
    img = Image.fromarray(grid)
    img.save(pgm_path)
    
    # Create YAML file
    yaml_content = f"""image: med_cim_map.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    
    print(f"✅ Map generated!")
    print(f"   PGM: {pgm_path}")
    print(f"   YAML: {yaml_path}")
    print(f"   Size: {WIDTH_PX}x{HEIGHT_PX} pixels")
    print(f"   World: {MAP_WIDTH}x{MAP_HEIGHT} meters")

if __name__ == '__main__':
    main()


