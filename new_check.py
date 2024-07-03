import cv2
import numpy as np

def load_yolo_labels(label_path, img_width, img_height):
    labels = []
    with open(label_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            class_id = int(float(parts[0]))
            center_x = float(parts[1]) * img_width
            center_y = float(parts[2]) * img_height
            width = float(parts[3]) * img_width
            height = float(parts[4]) * img_height
            x_min = int(center_x - width / 2)
            y_min = int(center_y - height / 2)
            x_max = int(center_x + width / 2)
            y_max = int(center_y + height / 2)
            labels.append((class_id, x_min, y_min, x_max, y_max))
    return labels

def draw_labels(image, labels):
    for label in labels:
        class_id, x_min, y_min, x_max, y_max = label
        color = (0, 255, 0)  # Green color for bounding box
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, 2)
        cv2.putText(image, str(class_id), (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

# Load image
image_path = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/dataset/images/train/15_X001_C011_1006_3.jpg'
label_path = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/dataset/labels/train/15_X001_C011_1006_3.txt'
image = cv2.imread(image_path)

if image is None:
    print(f"Failed to load image at {image_path}")
else:
    img_height, img_width = image.shape[:2]

    # Load and draw labels
    labels = load_yolo_labels(label_path, img_width, img_height)
    draw_labels(image, labels)

    # Display the image
    cv2.imshow('Labeled Image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()