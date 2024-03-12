import numpy as np
import cv2
from scipy.signal import convolve2d


def canny_edge_detection(image, low_threshold, high_threshold):
    
    # Apply Gaussian blur to reduce noise
    blurred = gaussian_blur(image, (5, 5), 0) #prerobit

    # Apply Sobel operator to calculate gradients
    gradient_x = sobel_operator(blurred, 1, 0, ksize=3) # prerobit
    gradient_y = sobel_operator(blurred, 0, 1, ksize=3)

    # Calculate gradient magnitude and direction
    gradient_magnitude = np.sqrt(gradient_x*gradient_x + gradient_y*gradient_y)
    gradient_direction = np.arctan2(gradient_y, gradient_x)

    # Apply non-maximum suppression to thin out edges
    suppressed = non_max_suppression(gradient_magnitude, gradient_direction)

    # Apply double thresholding to detect strong and weak edges
    edges = double_thresholding(suppressed, low_threshold, high_threshold)

    # Apply hysteresis thresholding to connect weak edges
    edges = hysteresis_thresholding(edges)

    return edges

def non_max_suppression(gradient_magnitude, gradient_direction):
    # Get the dimensions of the gradient magnitude
    height, width = gradient_magnitude.shape

    # Create an empty array to store the suppressed edges
    suppressed = np.zeros_like(gradient_magnitude)

    # Convert the gradient direction from radians to degrees
    gradient_direction = np.degrees(gradient_direction) % 180

    # Perform non-maximum suppression
    for i in range(1, height - 1):
        for j in range(1, width - 1):
            angle = gradient_direction[i, j]

            # Determine the neighboring pixels based on the gradient direction
            if (0 <= angle < 22.5) or (157.5 <= angle <= 180):
                neighbors = [gradient_magnitude[i, j - 1], gradient_magnitude[i, j + 1]]
            elif (22.5 <= angle < 67.5):
                neighbors = [gradient_magnitude[i - 1, j + 1], gradient_magnitude[i + 1, j - 1]]
            elif (67.5 <= angle < 112.5):
                neighbors = [gradient_magnitude[i - 1, j], gradient_magnitude[i + 1, j]]
            else:
                neighbors = [gradient_magnitude[i - 1, j - 1], gradient_magnitude[i + 1, j + 1]]

            # Suppress the edge if it is not the maximum among the neighbors
            if gradient_magnitude[i, j] >= max(neighbors):
                suppressed[i, j] = gradient_magnitude[i, j]

    return suppressed

def double_thresholding(gradient_magnitude, low_threshold, high_threshold):
    # Create an empty array to store the edges
    edges = np.zeros_like(gradient_magnitude)

    # Apply double thresholding
    strong_edges = gradient_magnitude >= high_threshold
    weak_edges = (gradient_magnitude >= low_threshold) & (gradient_magnitude < high_threshold)

    # Set strong edges to 255 (white)
    edges[strong_edges] = 255

    # Set weak edges to a lower intensity value (e.g., 50)
    edges[weak_edges] = 50

    return edges
   
def hysteresis_thresholding(edges):
    # Get the dimensions of the edges
    height, width = edges.shape

    # Create a copy of the edges
    thresholded = np.copy(edges)

    # Perform hysteresis thresholding
    for i in range(1, height - 1):
        for j in range(1, width - 1):
            if edges[i, j] == 255:  # Strong edge
                # Check the 8 neighboring pixels
                if (edges[i - 1, j - 1] == 50 or edges[i - 1, j] == 50 or edges[i - 1, j + 1] == 50 or
                        edges[i, j - 1] == 50 or edges[i, j + 1] == 50 or
                        edges[i + 1, j - 1] == 50 or edges[i + 1, j] == 50 or edges[i + 1, j + 1] == 50):
                    # Set the pixel to a weak edge
                    thresholded[i, j] = 50
                else:
                    # Set the pixel to a non-edge
                    thresholded[i, j] = 0

    return thresholded
 
def gaussian_blur(image, kernel_size, sigma): # Create a Gaussian kernel 
    kernel = np.fromfunction(lambda x, y: (1 / (2 * np.pi * sigma**2)) * np.exp(-pow((x - kernel_size[0]//2), 2) - pow((y - kernel_size[1]//2), 2) / (2 * sigma**2)) if sigma != 0 else 0, kernel_size)
    # Normalize the kernel
    kernel /= np.sum(kernel)

    # Get the dimensions of the image
    height, width = image.shape

    # Create an empty array to store the blurred image
    blurred = np.zeros_like(image)

    # Apply the Gaussian blur
    for i in range(kernel_size[0]//2, height - kernel_size[0]//2):
        for j in range(kernel_size[1]//2, width - kernel_size[1]//2):
            # Convolve the kernel with the image region
            region = image[i - kernel_size[0]//2:i + kernel_size[0]//2 + 1, j - kernel_size[1]//2:j + kernel_size[1]//2 + 1]
            blurred[i, j] = np.sum(region * kernel)

    return blurred

def sobel_operator(image, dx, dy, ksize): # Create the Sobel kernels 
    kernel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]]) 
    kernel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])
    # Apply the kernels to calculate gradients
    gradient_x = convolve2d(image, kernel_x, mode='same')
    gradient_y = convolve2d(image, kernel_y, mode='same')

    return gradient_x, gradient_y