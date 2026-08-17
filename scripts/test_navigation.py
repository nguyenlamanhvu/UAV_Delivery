import os
import numpy as np
import matplotlib.pyplot as plt
import heapq

def load_metadata():
    metadata = {}
    with open("maps/elevation_metadata.txt", "r") as f:
        for line in f:
            k, v = line.strip().split(": ")
            metadata[k] = float(v)
    return metadata

def heuristic(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def astar(elevation_grid, start, goal, z_flight, safety_margin=2.0):
    height, width = elevation_grid.shape
    
    pq = []
    heapq.heappush(pq, (heuristic(start, goal), 0, start, None))
    
    g_score = {start: 0}
    came_from = {}
    
    # 8-connected grid movements
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), 
                 (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    while pq:
        _, current_g, current, parent = heapq.heappop(pq)
        
        if current == goal:
            path = []
            curr = goal
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            return path[::-1]
            
        for dy, dx in neighbors:
            neighbor = (current[0] + dy, current[1] + dx)
            
            if 0 <= neighbor[0] < height and 0 <= neighbor[1] < width:
                # UAV flight height check:
                # The drone collides if the building/terrain elevation is too high
                obstacle_height = elevation_grid[neighbor[0], neighbor[1]]
                if obstacle_height >= (z_flight - safety_margin):
                    continue
                
                move_cost = np.sqrt(dy**2 + dx**2)
                tentative_g = current_g + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(pq, (f_score, tentative_g, neighbor, current))
                    
    return None

def main():
    if not os.path.exists("maps/campus_elevation.npy"):
        print("Elevation map not found! Please run scripts/export_elevation_map.py first.")
        return
        
    elevation_grid = np.load("maps/campus_elevation.npy")
    meta = load_metadata()
    
    def world_to_grid(x, y):
        col = int((x - meta['x_min']) / meta['resolution'])
        row = int((y - meta['y_min']) / meta['resolution'])
        # Clip to grid dimensions to prevent IndexError
        col = np.clip(col, 0, elevation_grid.shape[1] - 1)
        row = np.clip(row, 0, elevation_grid.shape[0] - 1)
        return row, col
        
    def grid_to_world(row, col):
        x = meta['x_min'] + col * meta['resolution']
        y = meta['y_min'] + row * meta['resolution']
        return x, y

    # Test settings
    # Coordinates in world frame (within bounds X: [-250, 100], Y: [-100, 200])
    start_world = (-150.0, 20.0)
    goal_world = (20.0, 120.0)
    
    start_grid = world_to_grid(*start_world)
    goal_grid = world_to_grid(*goal_world)
    
    print(f"Start world position: {start_world} has elevation height: {elevation_grid[start_grid[0], start_grid[1]]:.2f}m")
    print(f"Goal world position: {goal_world} has elevation height: {elevation_grid[goal_grid[0], goal_grid[1]]:.2f}m")
    
    # Let's try multiple flight altitudes: 110m (low altitude) and 160m (clear flight)
    for z_flight in [110.0, 160.0]:
        safety_margin = 2.0
        print(f"\n--- Planning at UAV Flight Altitude: {z_flight}m (Safety Margin: {safety_margin}m) ---")
        
        path_grid = astar(elevation_grid, start_grid, goal_grid, z_flight, safety_margin)
        
        if path_grid is None:
            print(f"Failed to find a path at Z={z_flight}m! Path is blocked.")
            continue
            
        path_world = [grid_to_world(r, c) for r, c in path_grid]
        print(f"Path successfully planned with {len(path_world)} waypoints.")
        
        # Plot results on top of the elevation heatmap
        plt.figure(figsize=(12, 8))
        im = plt.imshow(elevation_grid, cmap='viridis', origin='lower',
                        extent=[meta['x_min'], meta['x_max'], meta['y_min'], meta['y_max']])
        plt.colorbar(im, label="Elevation Height Z (meters)")
        
        path_x = [pt[0] for pt in path_world]
        path_y = [pt[1] for pt in path_world]
        plt.plot(path_x, path_y, color='red', linewidth=3, label=f'Planned Path (Z={z_flight}m)')
        plt.scatter(start_world[0], start_world[1], color='blue', s=100, zorder=5, label='Start')
        plt.scatter(goal_world[0], goal_world[1], color='cyan', s=100, zorder=5, label='Goal')
        
        plt.title(f"A* Pathfinding on 2.5D Campus Elevation Map (Z = {z_flight}m)")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig(f"maps/planned_path_{int(z_flight)}m.png", bbox_inches='tight')
        plt.close()
        
        print(f"Saved navigation test plot to maps/planned_path_{int(z_flight)}m.png")

if __name__ == "__main__":
    main()
