import os
import numpy as np
import matplotlib.pyplot as plt
import trimesh
from scipy.spatial import KDTree

def main():
    print("Loading campus visual mesh with trimesh...")
    # Load the GLTF mesh directly to get the correct scale, aspect ratio, and textures
    scene = trimesh.load('UAV_models/env/scene.gltf')
    mesh = scene.dump(concatenate=True) if isinstance(scene, trimesh.Scene) else scene
    
    print("Scaling and translating mesh...")
    mesh.apply_scale(1000.0)
    mesh.apply_translation([-100.0, 100.0, 2.5])
    
    # Extract colors
    print("Extracting vertex colors from GLTF textures...")
    color_visual = mesh.visual.to_color()
    vertex_colors = color_visual.vertex_colors[:, :3]  # Keep RGB
    
    vertices = mesh.vertices.copy()
    
    # Swap Y and Z to align with the Z-up frame
    print("Swapping Y and Z axes...")
    y_coords = vertices[:, 1].copy()
    z_coords = vertices[:, 2].copy()
    vertices[:, 1] = z_coords  # New Y is old Z
    vertices[:, 2] = y_coords  # New Z is old Y
    
    # Bound the crop region tightly around the actual campus main island bounds
    # to eliminate the surrounding empty dark green background
    x_min, x_max = -180.0, 40.0
    y_min, y_max = -30.0, 140.0
    
    base_ground_height = 94.0
    
    print(f"Generating high-resolution 2D map:")
    print(f"  X: [{x_min}, {x_max}] meters")
    print(f"  Y: [{y_min}, {y_max}] meters")
    
    resolution = 0.1  # 0.1 meters per cell (10cm/px) for extreme detail
    x_range = np.arange(x_min, x_max + resolution, resolution)
    y_range = np.arange(y_min, y_max + resolution, resolution)
    
    width = len(x_range)
    height = len(y_range)
    print(f"Grid size: {width}x{height} cells...")
    
    # Build 2D KDTree of mesh vertices for horizontal query
    print("Building 2D KDTree of mesh vertices...")
    kdtree_2d = KDTree(vertices[:, :2])
    
    # Create grid points in 2D
    xv, yv = np.meshgrid(x_range, y_range)
    grid_points_2d = np.stack([xv.ravel(), yv.ravel()], axis=-1)
    
    # Query nearest neighbors
    print("Querying nearest vertices for color and height mapping...")
    dists, indices = kdtree_2d.query(grid_points_2d, k=1)
    
    # Default ground color: a nice lawn green
    default_ground_color = np.array([45, 75, 45], dtype=np.uint8)
    
    # Build grids
    flat_colors = np.zeros((len(grid_points_2d), 3), dtype=np.uint8)
    flat_elevations = np.zeros(len(grid_points_2d))
    
    for i in range(len(grid_points_2d)):
        dist = dists[i]
        idx = indices[i]
        
        if dist <= 3.0:  # Proximity threshold
            flat_colors[i] = vertex_colors[idx]
            flat_elevations[i] = vertices[idx, 2]
        else:
            flat_colors[i] = default_ground_color
            flat_elevations[i] = base_ground_height
            
    # Reshape grids back to 2D
    color_grid = flat_colors.reshape((height, width, 3))
    elevation_grid = flat_elevations.reshape((height, width))
    
    # Save the elevation grid data (for pathfinding code)
    output_dir = "maps"
    os.makedirs(output_dir, exist_ok=True)
    np.save(os.path.join(output_dir, "campus_elevation.npy"), elevation_grid)
    
    # Save metadata
    with open(os.path.join(output_dir, "elevation_metadata.txt"), "w") as f:
        f.write(f"x_min: {x_min}\n")
        f.write(f"x_max: {x_max}\n")
        f.write(f"y_min: {y_min}\n")
        f.write(f"y_max: {y_max}\n")
        f.write(f"z_min: {base_ground_height}\n")
        f.write(f"z_max: {vertices[:, 2].max()}\n")
        f.write(f"resolution: {resolution}\n")
        
    print("Saved 2.5D elevation grid to maps/campus_elevation.npy")
    
    # Save the colored 2D satellite-style map
    plt.figure(figsize=(12, 10))
    plt.imshow(color_grid, origin='lower', extent=[x_min, x_max, y_min, y_max])
    plt.title("Campus 2D Textured Color Map (Top-Down Satellite View)")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True, alpha=0.3)
    plt.savefig(os.path.join(output_dir, "campus_color_map.png"), bbox_inches='tight')
    plt.close()
    print("Saved textured satellite-style map to maps/campus_color_map.png")
    
    # Save the combined map: Elevation Heatmap overlaid on the satellite colors
    print("Applying histogram equalization to visual heightmap...")
    flat_grid = elevation_grid.flatten()
    sort_indices = np.argsort(flat_grid)
    cdf = np.zeros_like(flat_grid)
    cdf[sort_indices] = np.linspace(0.0, 100.0, len(flat_grid))
    equalized_grid = cdf.reshape(elevation_grid.shape)
    
    plt.figure(figsize=(12, 10))
    plt.imshow(color_grid, origin='lower', extent=[x_min, x_max, y_min, y_max])
    im = plt.imshow(equalized_grid, cmap='viridis', origin='lower', 
                    extent=[x_min, x_max, y_min, y_max], alpha=0.5)
    plt.colorbar(im, label="Elevation Percentile Rank (%)")
    plt.title("Campus 2D Map (Elevation Heatmap Overlaid on Satellite Texture)")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True, alpha=0.3)
    plt.savefig(os.path.join(output_dir, "campus_elevation_heatmap.png"), bbox_inches='tight')
    plt.close()
    print("Saved combined elevation overlay map to maps/campus_elevation_heatmap.png")

if __name__ == "__main__":
    main()
