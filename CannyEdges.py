import TestCases
import cv2
import numpy as np

state_list = ["Hard Left", "Veer Left", "Straight", "Veer Right", "Hard Right"]
def rotate_and_crop(image):
    # Rotate image clockwise since our camera is sideways
    rotated_image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    # Crop the bottom half of the rotated image
    height, width = rotated_image.shape[:2]
    roi = rotated_image[int(height*0.65):height, :]
    return roi

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

def get_sidewalk_edge_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Return none if there are no contours
    if contours is None:
        return None
    # Otherwise, return the tallest contour
    else:
        return max(contours, key=lambda cnt: cv2.boundingRect(cnt)[3])  # The tallest contour

last_position = 3
def canny_edge_detection(image):
    global last_position
    height, width = image.shape[:2]
    # roi = rotate_and_crop(image)
    thresh_closed = generate_mask_using_edges(image)
    contours, _ = cv2.findContours(thresh_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)# Find the largest contour
    if contours:
        # print("# of contours", len(contours))
        cv2.drawContours(image, contours, -1, (255, 0, 0), 1)  # Draw contours in white (255) with thickness 2
        tallest_contour = max(contours, key=lambda cnt: cv2.boundingRect(cnt)[3])  # Compare by height

        # Get the bounding box of the largest contour
        x, y, w, h = cv2.boundingRect(tallest_contour)
        normalized_coordinate = x/width
        cur_position = int(normalized_coordinate * 5)
        
        if cur_position != last_position and normalized_coordinate > 0.1 and normalized_coordinate < 0.9: # Only update position if coordinat is within an expected range and the position is not the same
            print(state_list[cur_position])
            last_position = cur_position

        

        # Draw the bounding box on the original image
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw in green with thickness 2
        
    
    # Display the original image and the edges
    cv2.imshow(f'Detected Edge', image)
    # cv2.imshow('Canny Edges', edges)
    # cv2.imshow('Eroded Edges', eroded_edges)
    cv2.imshow('Mask', thresh_closed)


if __name__ == "__main__":
    TestCases.vmain(canny_edge_detection, resize_factor=0.3, video_path=r"RobotCam\WIN_20240402_14_58_52_Pro.mp4")
    # TestCases.main(canny_edge_detection)