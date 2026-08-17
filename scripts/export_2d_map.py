import os
import numpy as np
import matplotlib.pyplot as plt
import trimesh
from scipy.spatial import KDTree

def main():
    print("Loading campus mesh with trimesh...")
    scene = trimesh.load('models/campus_model.obj')
    mesh = scene.dump(concatenate=True) if isinstance(scene, trimesh.Scene) else scene
    
    # Apply the same transformations as campus.sdf
    print("Scaling and translating mesh...")
    mesh.apply_scale(1000.0)
    mesh.apply_translation([-100.0, 100.0, 2.5])
    
    vertices = mesh.vertices
    print(f"Building KDTree for {len(vertices)} vertices...")
    kdtree = KDTree(vertices)
    
    # Grid parameters
    # Let's cover X: [-200, 200], Y: [-200, 200] around the center of the campus (0, 0)
    z_altitude = 5.0  # meters (UAV flight height)
    resolution = 1.0  # meters per cell
    
    x_range = np.arange(-200.0, 200.0, resolution)
    y_range = np.arange(-200.0, 200.0, resolution)
    
    width = len(x_range)
    height = len(y_range)
    print(f"Generating a {width}x{height} 2D map at altitude Z = {z_altitude}m...")
    
    # Generate grid coordinates
    xv, yv = np.meshgrid(x_range, y_range)
    zv = np.full_like(xv, z_altitude)
    grid_points = np.stack([xv.ravel(), yv.ravel(), zv.ravel()], axis=-1)
    
    # Query KDTree
    print("Querying proximity tree for all grid cells...")
    dists, _ = kdtree.query(grid_points)
    
    # Reshape distances back to grid shape
    dists_grid = dists.reshape((height, width))
    
    # Define collision threshold (meters)
    # Since the mesh vertices are discrete, a threshold of 3.0-5.0 meters detects structures robustly
    collision_threshold = 4.0 
    occupancy_grid = (dists_grid <= collision_threshold).astype(np.uint8)
    
    # Save map data
    output_dir = "maps"
    os.makedirs(output_dir, exist_ok=True)
    np.save(os.path.join(output_dir, "campus_map.npy"), occupancy_grid)
    
    # Save parameters for reference in path planning
    with open(os.path.join(output_dir, "map_metadata.txt"), "w") as f:
        f.write(f"x_min: {x_range[0]}\n")
        f.write(f"x_max: {x_range[-1]}\n")
        f.write(f"y_min: {y_range[0]}\n")
        f.write(f"y_max: {y_range[-1]}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"z_altitude: {z_altitude}\n")
        
    print("Saved 2D map array to maps/campus_map.npy")
    
    # Save visual PNG map
    # We invert so obstacles are black (0) and free space is white (255)
    visual_map = (1 - occupancy_grid) * 255
    plt.figure(figsize=(10, 10))
    plt.imshow(visual_map, cmap='gray', origin='lower',
               extent=[x_range[0], x_range[-1], y_range[0], y_range[-1]])
    plt.title(f"Campus 2D Occupancy Map (Z = {z_altitude}m, thresh = {collision_threshold}m)")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, "campus_map.png"), bbox_inches='tight')
    plt.close()
    print("Saved visual map to maps/campus_map.png")

if __name__ == "__main__":
    main()
