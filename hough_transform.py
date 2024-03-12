import numpy as np
import math
import cv2

def hough_transform(image):
    # Define the maximum rho and theta values
    max_rho = int(math.sqrt(image.shape[0]**2 + image.shape[1]**2))
    max_theta = 180

    # Create the accumulator array
    accumulator = np.zeros((max_rho, max_theta))

    # Iterate over all pixels in the image
    for y in range(image.shape[0]):
        for x in range(image.shape[1]):
            # Check if the pixel is an edge pixel
            if image[y, x] > 0:
                # Iterate over all possible theta values
                for theta in range(max_theta):
                    # Convert theta to radians
                    theta_rad = math.radians(theta)

                    # Calculate rho for the current theta
                    rho = int(x * math.cos(theta_rad) + y * math.sin(theta_rad))

                    # Increment the accumulator at the corresponding (rho, theta) position
                    accumulator[rho, theta] += 1

    return accumulator

def detect_lines(accumulator, threshold):
    lines = []
    # Iterate over all positions in the accumulator
    for rho in range(accumulator.shape[0]):
        for theta in range(accumulator.shape[1]):
            # Check if the value at the current position exceeds the threshold
            if accumulator[rho, theta] >= threshold:
                # Calculate the corresponding theta in degrees
                theta_deg = math.degrees(theta)
                # Append the line parameters (rho, theta_deg) to the list of lines
                lines.append((rho, theta_deg))
    return lines

def draw_lines(image, lines):
    # Create a copy of the image to draw lines on
    image_with_lines = np.copy(image)
    # Iterate over all lines
    for rho, theta_deg in lines:
        # Convert theta from degrees to radians
        theta_rad = math.radians(theta_deg)
        # Calculate the sine and cosine of theta
        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        # Calculate two points on the line
        x1 = int(rho * cos_theta - image.shape[0] * sin_theta)
        y1 = int(rho * sin_theta + image.shape[0] * cos_theta)
        x2 = int(rho * cos_theta + image.shape[0] * sin_theta)
        y2 = int(rho * sin_theta - image.shape[0] * cos_theta)
        # Draw the line on the image
        cv2.line(image_with_lines, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return image_with_lines

