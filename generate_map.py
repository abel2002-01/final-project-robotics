#!/usr/bin/env python3
"""
Generate Office Map from World Definition (Pure Python - no numpy required)
Creates a proper occupancy grid map for Nav2 navigation based on the office_world.sdf layout.

Office dimensions (from SDF):
- Outer walls: x=-1 to 15, y=-1 to 9 (16m x 10m)
- Resolution: 0.05m per pixel (5cm)
"""

import os

# Map parameters
RESOLUTION = 0.05  # meters per pixel
ORIGIN_X = -2.0
ORIGIN_Y = -2.0
MAP_WIDTH_M = 20.0   # meters (x: -2 to 18)
MAP_HEIGHT_M = 14.0  # meters (y: -2 to 12)

MAP_WIDTH_PX = int(MAP_WIDTH_M / RESOLUTION)   # 400 pixels
MAP_HEIGHT_PX = int(MAP_HEIGHT_M / RESOLUTION) # 280 pixels

# PGM values: 254=free, 0=occupied, 205=unknown
FREE = 254
OCCUPIED = 0
UNKNOWN = 205


def world_to_pixel(x, y):
    """Convert world coordinates to pixel coordinates."""
    px = int((x - ORIGIN_X) / RESOLUTION)
    py = int((y - ORIGIN_Y) / RESOLUTION)
    return px, py


def draw_wall(map_grid, x1, y1, x2, y2, thickness=0.15):
    """Draw a wall (occupied area) on the map."""
    half_t = thickness / 2
    
    min_x = min(x1, x2) - half_t
    max_x = max(x1, x2) + half_t
    min_y = min(y1, y2) - half_t
    max_y = max(y1, y2) + half_t
    
    px1, py1 = world_to_pixel(min_x, min_y)
    px2, py2 = world_to_pixel(max_x, max_y)
    
    px1 = max(0, min(px1, MAP_WIDTH_PX - 1))
    px2 = max(0, min(px2, MAP_WIDTH_PX - 1))
    py1 = max(0, min(py1, MAP_HEIGHT_PX - 1))
    py2 = max(0, min(py2, MAP_HEIGHT_PX - 1))
    
    for y in range(py1, py2 + 1):
        for x in range(px1, px2 + 1):
            map_grid[y][x] = OCCUPIED


def draw_box(map_grid, cx, cy, width, height):
    """Draw a box obstacle (furniture) on the map."""
    half_w = width / 2
    half_h = height / 2
    
    px1, py1 = world_to_pixel(cx - half_w, cy - half_h)
    px2, py2 = world_to_pixel(cx + half_w, cy + half_h)
    
    px1 = max(0, min(px1, MAP_WIDTH_PX - 1))
    px2 = max(0, min(px2, MAP_WIDTH_PX - 1))
    py1 = max(0, min(py1, MAP_HEIGHT_PX - 1))
    py2 = max(0, min(py2, MAP_HEIGHT_PX - 1))
    
    for y in range(py1, py2 + 1):
        for x in range(px1, px2 + 1):
            map_grid[y][x] = OCCUPIED


def set_region(map_grid, px1, py1, px2, py2, value):
    """Set a rectangular region to a specific value."""
    px1 = max(0, min(px1, MAP_WIDTH_PX - 1))
    px2 = max(0, min(px2, MAP_WIDTH_PX - 1))
    py1 = max(0, min(py1, MAP_HEIGHT_PX - 1))
    py2 = max(0, min(py2, MAP_HEIGHT_PX - 1))
    
    for y in range(py1, py2 + 1):
        for x in range(px1, px2 + 1):
            map_grid[y][x] = value


def generate_office_map():
    """Generate the office occupancy grid map."""
    # Initialize map as free space (2D list)
    map_grid = [[FREE for _ in range(MAP_WIDTH_PX)] for _ in range(MAP_HEIGHT_PX)]
    
    # Mark area outside office as unknown
    px_left, _ = world_to_pixel(-1, 0)
    px_right, _ = world_to_pixel(15, 0)
    _, py_bottom = world_to_pixel(0, -1)
    _, py_top = world_to_pixel(0, 9)
    
    # Outside areas - unknown
    set_region(map_grid, 0, 0, px_left - 1, MAP_HEIGHT_PX - 1, UNKNOWN)  # Left
    set_region(map_grid, px_right + 3, 0, MAP_WIDTH_PX - 1, MAP_HEIGHT_PX - 1, UNKNOWN)  # Right
    set_region(map_grid, 0, 0, MAP_WIDTH_PX - 1, py_bottom - 1, UNKNOWN)  # Bottom
    set_region(map_grid, 0, py_top + 3, MAP_WIDTH_PX - 1, MAP_HEIGHT_PX - 1, UNKNOWN)  # Top
    
    # ==================== OUTER WALLS ====================
    wall_thickness = 0.15
    
    # North wall: y=9, x=-1 to 15
    draw_wall(map_grid, -1, 9, 15, 9, wall_thickness)
    
    # South wall left: y=-1, x=-1 to 4
    draw_wall(map_grid, -1, -1, 4, -1, wall_thickness)
    
    # South wall right: y=-1, x=8 to 15
    draw_wall(map_grid, 8, -1, 15, -1, wall_thickness)
    
    # West wall: x=-1, y=-1 to 9
    draw_wall(map_grid, -1, -1, -1, 9, wall_thickness)
    
    # East wall: x=15, y=-1 to 9
    draw_wall(map_grid, 15, -1, 15, 9, wall_thickness)
    
    # ==================== INTERIOR WALLS ====================
    int_wall_thickness = 0.12
    
    # Storage wall: y=6.5, x=3 to 11
    draw_wall(map_grid, 3, 6.5, 11, 6.5, int_wall_thickness)
    
    # Office wall: x=10, y=-1.5 to 5.5
    draw_wall(map_grid, 10, -1.5, 10, 5.5, int_wall_thickness)
    
    # ==================== BATHROOM 1 WALLS ====================
    draw_wall(map_grid, 6.5, 2.5, 10.25, 2.5, int_wall_thickness)
    draw_wall(map_grid, 6.75, -0.5, 9.25, -0.5, int_wall_thickness)
    draw_wall(map_grid, 10.25, -0.5, 10.25, 2.5, int_wall_thickness)
    
    # ==================== BATHROOM 2 WALLS ====================
    draw_wall(map_grid, 11.75, 2.5, 14.25, 2.5, int_wall_thickness)
    draw_wall(map_grid, 12, -0.5, 14, -0.5, int_wall_thickness)
    draw_wall(map_grid, 11.75, -0.5, 11.75, 2.5, int_wall_thickness)
    
    # ==================== FURNITURE (Obstacles) ====================
    # Reception area
    draw_box(map_grid, 1.5, 1.8, 1.2, 0.8)   # Reception desk
    draw_box(map_grid, 0.8, 3.8, 1.5, 0.8)   # Couch
    draw_box(map_grid, -0.7, 1.0, 0.5, 0.8)  # Cabinet
    
    # Break area
    draw_box(map_grid, 7, 3.5, 1.2, 0.8)     # Break table
    
    # Storage area
    draw_box(map_grid, 1.5, 8.9, 1.0, 0.4)   # Shelf 1
    draw_box(map_grid, 4.5, 8.9, 1.0, 0.4)   # Shelf 2
    draw_box(map_grid, 8, 7.8, 0.5, 0.8)     # Cabinet
    
    # Office area
    draw_box(map_grid, 12.5, 1.5, 1.2, 0.8)  # Desk 1
    draw_box(map_grid, 12.5, 4.5, 1.2, 0.8)  # Desk 2
    draw_box(map_grid, 14, 3.5, 0.4, 1.0)    # Bookshelf
    draw_box(map_grid, 10.5, 0.5, 0.5, 0.8)  # Cabinet 1
    draw_box(map_grid, 10.5, 4.0, 0.5, 0.8)  # Cabinet 2
    
    # Bathroom furniture
    draw_box(map_grid, 7.5, 1.2, 0.5, 0.5)   # Sink 1
    draw_box(map_grid, 9.5, 1.2, 0.5, 0.7)   # Toilet 1
    draw_box(map_grid, 12.5, 1.2, 0.5, 0.7)  # Toilet 2
    draw_box(map_grid, 13.5, 1.2, 0.5, 0.5)  # Sink 2
    
    return map_grid


def save_pgm(map_grid, filename):
    """Save map as PGM file (P5 binary format)."""
    height = len(map_grid)
    width = len(map_grid[0])
    
    with open(filename, 'wb') as f:
        # PGM header
        f.write(f"P5\n{width} {height}\n255\n".encode())
        # Flip vertically for ROS convention (origin at bottom-left)
        for y in range(height - 1, -1, -1):
            row_bytes = bytes(map_grid[y])
            f.write(row_bytes)
    
    print(f"Saved PGM map: {filename} ({width}x{height} pixels)")


def save_yaml(yaml_filename, pgm_filename, resolution, origin_x, origin_y):
    """Save map YAML metadata file."""
    content = f"""image: {os.path.basename(pgm_filename)}
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(yaml_filename, 'w') as f:
        f.write(content)
    
    print(f"Saved YAML metadata: {yaml_filename}")


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(script_dir, 'delivery_robot', 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    
    print("Generating office map...")
    map_grid = generate_office_map()
    
    pgm_file = os.path.join(maps_dir, 'office_map.pgm')
    yaml_file = os.path.join(maps_dir, 'office_map.yaml')
    
    save_pgm(map_grid, pgm_file)
    save_yaml(yaml_file, pgm_file, RESOLUTION, ORIGIN_X, ORIGIN_Y)
    
    print(f"\nMap generated successfully!")
    print(f"  Resolution: {RESOLUTION}m/pixel")
    print(f"  World size: {MAP_WIDTH_M}m x {MAP_HEIGHT_M}m")
    print(f"  Pixel size: {MAP_WIDTH_PX} x {MAP_HEIGHT_PX}")
    print(f"  Origin: ({ORIGIN_X}, {ORIGIN_Y})")
    print(f"\nWaypoints (pixel coordinates):")
    print(f"  Reception: (2, 1) -> pixel {world_to_pixel(2, 1)}")
    print(f"  Storage:   (4, 7.5) -> pixel {world_to_pixel(4, 7.5)}")
    print(f"  Office:    (12.5, 3) -> pixel {world_to_pixel(12.5, 3)}")


if __name__ == '__main__':
    main()
