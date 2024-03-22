import numpy as np
import math
import cv2

def hough_transform(canny, threshold=150):

    # Define Hough space
    height, width = canny.shape
    max_rho = int(np.sqrt(height**2 + width**2))
    accumulator = np.zeros((2 * max_rho, 180))
    
    # Compute Hough transform
    for y in range(height):
        for x in range(width):
            if canny[y, x] != 0:
                for theta in range(0, 180):
                    rho = int(x * np.cos(np.deg2rad(theta)) + y * np.sin(np.deg2rad(theta)))
                    accumulator[rho + max_rho, theta] += 1
    
    # Find lines
    lines = []
    for rho in range(accumulator.shape[0]):
        for theta in range(accumulator.shape[1]):
            if accumulator[rho, theta] > threshold:
                lines.append((rho - max_rho, theta))
    
    return lines