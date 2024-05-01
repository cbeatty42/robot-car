import time
import TestCases
import cv2
import numpy as np
    

lower_green = 40  # Green starts at around 40 in OpenCV's HSV scale
upper_green = 70  # Green ends at around 70 in OpenCV's HSV scale

last_position = 3
last_coordinate = 0.5
estimated_position = 200
last_time = 0

def get_setpoint(norm_coordinate):
    cur_position = "D" + "{:.3f}".format(norm_coordinate*2-1)
    print(cur_position)
    return cur_position

def check_green_pixels(hue_channel, width):
    global lower_green, upper_green
    # Define the region of interest (1/8th of the width)
    roi_width = width // 8
    roi_start = width - roi_width

    # Extract the region of interest
    roi = hue_channel[:, roi_start:]
    # Count the number of pixels with hue values in the green range
    green_pixels = np.sum((hue_channel >= lower_green) & (hue_channel <= upper_green))

    # Calculate the total number of pixels
    total_pixels = hue_channel.size

    # Check if at least 30% of the pixels are in the green range
    if green_pixels / total_pixels >= 0.15:
        return True
    else:
        return False

def generate_mask_using_edges(image):
    # Apply Gaussian blur to the image to reduce noise
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    
    # Perform Canny edge detection
    edges = cv2.Canny(blurred, 75, 225)
    
    # Dilate the edges to enhance and connect them
    dialate_kernel = np.ones((3, 3), np.uint8)
    dilated_edges = cv2.dilate(edges, dialate_kernel, iterations=1)
    vertical_kernel = np.array([[0, 1, 0],
                                [0, 1, 0],
                                [0, 1, 0],
                                [0, 1, 0],
                                [0, 1, 0]], dtype=np.uint8)
    eroded_edges = cv2.erode(dilated_edges, vertical_kernel, iterations=2)

    
    dialate_kernel = np.ones((3, 3), np.uint8)
    dilated_edges = cv2.dilate(eroded_edges, dialate_kernel, iterations=2)

    close_kernel = np.ones((5, 5), np.uint8)
    thresh_closed = cv2.morphologyEx(dilated_edges, cv2.MORPH_CLOSE, close_kernel)
    return thresh_closed


def generate_mask_from_green(hue_channel):
    global lower_green, upper_green    
    mask = cv2.inRange(hue_channel, lower_green, upper_green)
    return mask


def sidewalk_edge_detection(image, output_queue=None):
    global last_position, last_coordinate, estimated_position

    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    hue, saturation, value = cv2.split(hsv)

    height, width = image.shape[:2]

    # Update timestamp and delta t
    cur_time = time.time()
    if check_green_pixels(hue, width):       
        mask = generate_mask_from_green(hue)
    else:
        mask = generate_mask_using_edges(value)
    
    # If you're on RBPI, comment out the line for windows and vice-versa

    # contour_img, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Find the largest contour on raspberry pi
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)# Find the largest contour on windows
    if contours:
        # print("# of contours", len(contours))
        cv2.drawContours(image, contours, -1, (255, 0, 0), 1)  # Draw contours in white (255) with thickness 2
        tallest_contour = max(contours, key=lambda cnt: cv2.boundingRect(cnt)[3])  # Compare by height
        # Get the bounding box of the largest contour
        x, y, w, h = cv2.boundingRect(tallest_contour)
        normalized_height = h/height        

        normalized_coordinate = x/width
        delta_c = normalized_coordinate - last_coordinate

        volatility_metric = (1-abs(delta_c))**7
        measurement_strength = normalized_height
        alpha = (volatility_metric * measurement_strength)/2

        # print(volatility_metric, normalized_height, alpha)

        estimated_position += alpha * (x - estimated_position)

        last_coordinate = normalized_coordinate

        cur_position = get_setpoint(normalized_coordinate)
        if cur_position != last_position and normalized_coordinate > 0.03 and normalized_coordinate < 0.97 and normalized_height >= 0.2 : # Only update position if coordinate is within an expected range, it is tall enough, and the position is not the same as prev
            print(65 - int(50 * (normalized_coordinate*2-1)), 65 + int(50*(normalized_coordinate*2-1)))
            # print(state_list[cur_position])
            if output_queue is not None:
                output_queue.put(cur_position)
            last_position = cur_position

        

        # Draw the bounding box on the original image
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)  # Draw in green with thickness 2
        cv2.rectangle(image, (x, y), (x + 1, y + h), (0, 255, 0), 2)  # Draw in green with thickness 2
        # Draw the bounding box on the original image
    cv2.rectangle(image, (int(estimated_position), 0), (int(estimated_position)+ 1, height), (0, 0, 255), 4)  # Draw in green with thickness 2
    
    # Display the original image and the edges
    cv2.imshow('Detected Edge', image)

    cv2.waitKey(25)
    # cv2.imshow('Eroded Edges', eroded_edges)
    # cv2.imshow('Mask', thresh_closed)


if __name__ == "__main__":
    TestCases.vmain(lambda image:sidewalk_edge_detection(image), resize_factor=0.3, video_path=r"RobotCam\T5.mp4")
    # TestCases.main(canny_edge_detection)