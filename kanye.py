import numpy as np

def canny_edge_detection(image, low_threshold, high_threshold):
    
    # Apply Gaussian blur to reduce noise
    blurred = gaussian_blur(image)

    gradient_magnitude, gradient_direction = sobel_operator(blurred)

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
 
def gaussian_blur(img): # Create a Gaussian kernel 
    for i in range(1,img.shape[0]-1):
        for j in range(1,img.shape[1]-1):
            img[i,j] = (68*img[i,j] + img[i-1,j] + img[i+1,j] + img[i,j-1] + img[i,j+1])/72.0
    return img

def sobel_operator(image): # Create the Sobel kernels 
    Gx = np.array([[1.0, 0.0, -1.0], [2.0, 0.0, -2.0], [1.0, 0.0, -1.0]])
    Gy = np.array([[1.0, 2.0, 1.0], [0.0, 0.0, 0.0], [-1.0, -2.0, -1.0]])
    [rows, columns] = np.shape(image)  # we need to know the shape of the input grayscale image
    gradient_magnitude = np.zeros(shape=(rows, columns))  # initialization of the output image array (all elements are 0)
    gradient_direction = np.zeros(shape=(rows, columns))  # initialization of the output image array (all elements are 0)

    # Now we "sweep" the image in both x and y directions and compute the output
    for i in range(rows - 2):
        for j in range(columns - 2):
            gx = np.sum(np.multiply(Gx, image[i:i + 3, j:j + 3]))  # x direction
            gy = np.sum(np.multiply(Gy, image[i:i + 3, j:j + 3]))  # y direction
            gradient_magnitude[i + 1, j + 1] = np.sqrt(gx ** 2 + gy ** 2)  # calculate the "hypotenuse"
            gradient_direction[i + 1, j + 1] = np.arctan2(gy, gx)  # calculate the gradient direction
    return [gradient_magnitude, gradient_direction]
