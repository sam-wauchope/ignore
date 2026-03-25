import open3d as o3d
import numpy as np
import serial
import time
import math

# --- 1. Setup ---
try:
    ser = serial.Serial('COM4', 115200, timeout=0.1)
    time.sleep(2)
except Exception as e:
    print(f"Failed to open Serial Port: {e}")
    exit()

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="One Rotation", width=1024, height=768)

pcd = o3d.geometry.PointCloud()
line_set = o3d.geometry.LineSet()

# Add the XYZ axis helper immediately
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500.0, origin=[0, 0, 0])
vis.add_geometry(mesh_frame)

points = []
lines = []
geometries_added = False # This flag prevents the Open3D warnings!
point_idx = 0

print("Start Scanning")

# Open the file to save our point cloud
f = open('tof_radar.xyz', 'w') 

# --- 2. Live Scan Loop ---
try:
    while vis.poll_events():
        if ser.in_waiting > 0:
            raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
            data_parts = raw_line.split(',')
            
            if len(data_parts) == 5:
                try:
                    distance = int(data_parts[1].strip())
                    
                    # Calculate angle (0, 45, 90, 135...)
                    angle_deg = point_idx * 45
                    angle_rad = math.radians(angle_deg)
                    
                    # Calculate Cartesian coordinates
                    x = 0.0
                    y = distance * math.cos(angle_rad)
                    z = distance * math.sin(angle_rad)
                    
                    # 1. Save to RAM
                    points.append([x, y, z])
                    
                    # 2. Save pure data to .xyz file
                    f.write(f"{x:.2f} {y:.2f} {z:.2f}\n")
                    f.flush()
                    
                    # 3. Line Connection Logic
                    if point_idx > 0:
                        lines.append([point_idx - 1, point_idx]) # Connect to previous point
                    if point_idx == 7: 
                        lines.append([7, 0]) # The 8th point connects back to the start!
                        
                    # 4. Update Open3D Arrays
                    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
                    line_set.points = o3d.utility.Vector3dVector(np.asarray(points))
                    if len(lines) > 0:
                        line_set.lines = o3d.utility.Vector2iVector(np.asarray(lines))
                        colors = [[0.0, 1.0, 0.0] for _ in range(len(lines))]
                        line_set.colors = o3d.utility.Vector3dVector(colors)

                    # --- THE BYPASS TRICK ---
                    # Only add geometries to the visualizer AFTER we have real data
                    if not geometries_added:
                        vis.add_geometry(pcd)
                        vis.add_geometry(line_set)
                        geometries_added = True
                    else:
                        vis.update_geometry(pcd)
                        vis.update_geometry(line_set)
                    
                    print(f"Point {point_idx}/8 | Angle: {angle_deg:3d}° | Dist: {distance:4d}mm")
                    
                    point_idx += 1
                    
                    # 5. Stop Condition
                    if point_idx == 8:
                        vis.reset_view_point(True) # Auto-zoom camera to fit the shape
                        print("\nRotation complete! Data saved to 'tof_radar.xyz'.")
                        break # Break out of the UART reading loop

                except ValueError:
                    pass
        
        vis.update_renderer()

    # --- 3. Post-Scan Viewing ---
    if point_idx == 8:
        print("The 3D window is still open for inspection. Close the window to exit the script.")
        vis.run() # This blocks the script from ending so you can rotate and look at the final shape

except KeyboardInterrupt:
    print("\nScan interrupted by user.")
finally:
    f.close()
    ser.close()
    vis.destroy_window()
