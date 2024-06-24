from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np

import matplotlib.pyplot as plt
import cv2

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
        grid_size = 50  # Define your grid size
        
        grid = image_to_grid(processed_image, grid_size)
        
        # Plot the grid
        plt.imshow(grid, cmap='gray', interpolation='nearest')
        plt.show()


        

       
    except KeyboardInterrupt:
            print("Simulation interrupted by user.")
        
    finally:
            sim.stopSimulation()
            print("Simulation stopped and cleaned up.")
    
    
    
    
if __name__ == '__main__':
    main()
