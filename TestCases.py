import os

import cv2


def main(process_image, resize_factor=0.3):
    # Path to the directory containing the images
    images_dir = r'Robotics\Images'

    # Get the list of files in the directory
    image_files = os.listdir(images_dir)
    
    for image_file in image_files:
        
        if not(image_file.endswith('.jpg') or image_file.endswith('.jpeg') or image_file.endswith('.png')):
            print("File", image_file, "is not a known image format")
            continue
        # Read the image
        image_path = os.path.join(images_dir, image_file)
        img = cv2.imread(image_path)
        
        img = cv2.resize(img, (0, 0), fx=resize_factor, fy=resize_factor) # Resize the image for visibility for users and to decrease resource usage

        if img is None:
            # print(f"Could not read image: {image_path}")
            continue

        process_image(img)
        
        cv2.waitKey(0)

        cv2.destroyAllWindows()
def vmain(process_image, resize_factor=0.3, video_path=r"ConcreteSidewalk.mp4"):
    # Abre el video
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error al abrir el video")
        return
    counter = 0
    while True:
        # Lee un frame
        ret, frame = cap.read()
        if not ret:
            break
        
        counter += 1
        if counter % 2 != 0:
            continue

        frame = cv2.resize(frame, (0, 0), fx=resize_factor, fy=resize_factor) # Resize the image for visibility for users and to decrease resource usage

        process_image(frame)
        # Detiene el programa si se presiona la tecla 'q'
        if cv2.waitKey(15) & 0xFF == ord('q'):
            break

    # Libera el objeto de captura y cierra todas las ventanas
    cap.release()
    cv2.destroyAllWindows()