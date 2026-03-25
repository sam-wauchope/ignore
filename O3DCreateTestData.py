import open3d as o3d
import numpy as np
import serial
import time
import math


try:
    ser = serial.Serial('COM4', 115200, timeout=0.1)
    time.sleep(2) 
except Exception as e:
    print(f"Failed to open Serial Port: {e}")
    exit()

# --- 2. Open3D Real-Time Visualizer Setup ---
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="lab 8", width=1024, height=768)

# Make points large enough to see easily
render_opt = vis.get_render_option()
render_opt.point_size = 5.0
render_opt.background_color = np.asarray([0.1, 0.1, 0.1]) # Dark gray background

# Initialize empty geometry objects
pcd = o3d.geometry.PointCloud()
line_set = o3d.geometry.LineSet()

# Add them to the visualizer
vis.add_geometry(pcd)
vis.add_geometry(line_set)

# --- 3. State Variables ---
points = [[0.0, 0.0, 0.0]]
lines = [[0, 0]]

angle_step_deg = 45
current_angle_deg = 0
x_depth_position = 0
x_step_size = 50  # How far forward the scanner moves down the hallway per revolution
point_idx = 1     # Keeps track of total points collected

pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
line_set.points = o3d.utility.Vector3dVector(np.asarray(points))
line_set.lines = o3d.utility.Vector2iVector(np.asarray(lines))
vis.update_geometry(pcd)
vis.update_geometry(line_set)

print("Starting live scan... Press 'Q' or 'Esc' in the 3D window to quit.")

# --- 4. Main Real-Time Loop ---
try:
    # Keep running as long as the 3D window is open
    while vis.poll_events():
        new_data_received = False
        
        # Check if the microcontroller sent a new line
        if ser.in_waiting > 0:
            raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
            data_parts = raw_line.split(',')
            
            if len(data_parts) == 5:
                try:
                    # Extract distance (Index 1 based on your sprintf: RangeStatus, Distance, ...)
                    distance = int(data_parts[1].strip())
                    
                    # Convert polar to Cartesian (X = depth, Y = horizontal, Z = vertical)
                    angle_rad = math.radians(current_angle_deg)
                    x = float(x_depth_position)
                    y = distance * math.cos(angle_rad)
                    z = distance * math.sin(angle_rad)
                    
                    # Add new point to our list
                    points.append([x, y, z])
                    
                    # --- DYNAMIC LINE CONNECTION LOGIC ---
                    # 1. Connect current point to the previous point in the current ring
                    if point_idx % 8 != 0: 
                        lines.append([point_idx - 1, point_idx])
                        
                    # 2. Close the ring (Connect the 8th point back to the 1st point of the ring)
                    if point_idx % 8 == 7:
                        lines.append([point_idx, point_idx - 7])
                        
                    # 3. Connect to the previous slice (Stitching the hallway together)
                    # We can only do this once we have finished at least one full ring of 8 points
                    if point_idx >= 8:
                        lines.append([point_idx, point_idx - 8])
                        
                    new_data_received = True
                    
                    print(f"Angle: {current_angle_deg:3d} | Dist: {distance:4d} | Pts Collected: {point_idx + 1}")
                    
                    # Prepare for the next point
                    point_idx += 1
                    current_angle_deg += angle_step_deg
                    
                    # If we complete a 360 circle, reset the angle and move forward in the X axis
                    if current_angle_deg >= 360:
                        current_angle_deg = 0
                        x_depth_position += x_step_size

                except ValueError:
                    pass # Ignore broken packets
        
        # --- 5. Update the 3D View ---
        if new_data_received:
            # Update PointCloud and LineSet data (Keep your existing code here)
            pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
            line_set.points = o3d.utility.Vector3dVector(np.asarray(points))
            if len(lines) > 0:
                line_set.lines = o3d.utility.Vector2iVector(np.asarray(lines))
            
            colors = [[0.0, 1.0, 0.0] for _ in range(len(lines))]
            line_set.colors = o3d.utility.Vector3dVector(colors)
            
            vis.update_geometry(pcd)
            vis.update_geometry(line_set)
            
            # ---> ADD THIS CAMERA FIX <---
            # Only reset the camera angle once, right after the first complete 8-point ring is drawn.
            if point_idx == 9: 
                vis.reset_view_point(True)
                
        # Update the screen renderer
        vis.update_renderer()

except KeyboardInterrupt:
    print("\nScan interrupted by user.")
finally:
    ser.close()
    vis.destroy_window()
    print("Cleaned up resources. Scan ended.")
                                    
    
 
