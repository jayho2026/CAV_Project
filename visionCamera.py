from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np

import matplotlib.pyplot as plt
import cv2
import json

def fetch_image_from_sensor(sim, vision_sensor_handle):
    # Ensure the vision sensor is handled if needed
    sim.handleVisionSensor(vision_sensor_handle)

    # Fetch the image data
    image, resolution = sim.getVisionSensorImg(vision_sensor_handle)
    
    # Convert the byte data to an array
    if image:
            
        print("Data Check:", np.frombuffer(image, dtype=np.uint8).sum())  # Should not be zero if image has content

        # Determine the number of channels based on the options used
        channels = 3  # Default to RGB
        image_array = np.frombuffer(image, dtype=np.uint8).reshape((resolution[1], resolution[0], channels))

        # Display the image using Matplotlib
        plt.imshow(image_array)
        plt.show()

        
        return cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR for OpenCV compatibility
    
    else:
        raise Exception("Failed to retrieve image from vision sensor")


def preprocess_image(image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    
    
    
    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    
    
    # Threshold the image
    _, binary = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY)
    
    plt.imshow(gray, cmap='gray')
    plt.show()
    
    # Apply morphological operations
    kernel = np.ones((5, 5), np.uint8)
    cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
    
    
    
    return cleaned

def find_contours(binary_image):
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def image_to_grid(binary_image, grid_size):
    height, width = binary_image.shape
    cell_height = height // grid_size
    cell_width = width // grid_size
    grid = np.zeros((grid_size, grid_size), dtype=int)

    for i in range(grid_size):
        for j in range(grid_size):
            cell = binary_image[i * cell_height:(i + 1) * cell_height, j * cell_width:(j + 1) * cell_width]
            if np.sum(cell) > 0:  # Adjust this threshold based on your needs
                grid[i, j] = 1  # Mark as obstacle
    
    print(grid)
    return grid

def save_to_json(obstacles, filename='grid.json'):
    # Open a file and use json.dump to write the list of dictionaries to the file
    with open(filename, 'w') as f:
        json.dump(obstacles, f, indent=4)  # Pretty print the data for readability

def convert_to_simulation_coordinates(grid):
    scale_factor = 0.0254 #0.127  # meters per cell
    origin_shift = grid.shape[0] // 2  # Assuming grid is square
    obstacles = []

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] == 0:  # Assuming '0' represents an obstacle
                x = ((j - origin_shift) * scale_factor) - 0.04
                y = ((i - origin_shift) * scale_factor) - 0.04
                obstacles.append({"x": x, "y": y, "size": scale_factor})
    
    save_to_json(obstacles)            
                
    print(obstacles)
                
    return obstacles

def visualize_obstacles(grid):
    obstacles = convert_to_simulation_coordinates(grid)
    # Prepare to plot
    fig, ax = plt.subplots()
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_title('Simulation Grid Visualization')
    ax.set_xlabel('Meters (X)')
    ax.set_ylabel('Meters (Y)')
    
    ax.scatter([-2.5], [-2.5], color='blue', s=100, marker='o', label='Marked Point')


    # Plot each obstacle
    for obstacle in obstacles:
        rect = plt.Rectangle((obstacle['x'], obstacle['y']), obstacle['size'], obstacle['size'], color='red')
        ax.add_patch(rect)

    # Adding grid lines for reference
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax.axhline(y=0, color='k')
    ax.axvline(x=0, color='k')

    plt.show()

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    topVision = sim.getObjectHandle('/Vision_sensor')
    
    print(topVision)
    
    sim.startSimulation()
    
    sim.handleVisionSensor(topVision)

    # Fetch the image data
    image, resolution = sim.getVisionSensorImg(topVision)
    
    try:
        
        image = fetch_image_from_sensor(sim, topVision)
        
                
        # Preprocess the image
        processed_image = preprocess_image(image)
        
        # Optionally find contours
        contours = find_contours(processed_image)
        
        
        # Convert to grid
        grid_size = 250  # Define your grid size
        
        grid = image_to_grid(processed_image, grid_size)
        
        visualize_obstacles(grid)
                
        # Plot the grid
        #plt.imshow(grid, cmap='gray', interpolation='nearest')
        #plt.show()


        

       
    except KeyboardInterrupt:
            print("Simulation interrupted by user.")
        
    finally:
            sim.stopSimulation()
            print("Simulation stopped and cleaned up.")
    
    
    
    
if __name__ == '__main__':
    main()
