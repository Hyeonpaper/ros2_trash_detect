import cv2
import os

def draw_bounding_boxes(image, labels):
    for label in labels:
        cls, x_center, y_center, width, height = map(float, label.split())

        # Convert YOLO format (normalized) to pixel coordinates
        img_h, img_w = image.shape[:2]
        x_center *= img_w
        y_center *= img_h
        width *= img_w
        height *= img_h

        # Convert to corners
        x1 = int(x_center - width / 2)
        y1 = int(y_center - height / 2)
        x2 = int(x_center + width / 2)
        y2 = int(y_center + height / 2)

        # Draw rectangle on image
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    return image

# 경로 설정
#image_dir = '/Users/ijonghyeon/Downloads/생활 폐기물 이미지/resize_images/val'
image_dir = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/datasets/images/val'
#label_dir = '/Users/ijonghyeon/Downloads/생활 폐기물 이미지/resize_labels/val'
label_dir = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/datasets/labels/val'
output_image_dir = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/dataset/check9'

# 디렉토리가 존재하지 않으면 생성
if not os.path.exists(output_image_dir):
    os.makedirs(output_image_dir)

# 이미지와 라벨링 파일 리스트를 가져옴
image_files = [f for f in os.listdir(image_dir) if f.endswith('.jpg') or f.endswith('.png')]

for image_file in image_files:
    # 이미지 경로와 라벨링 경로 설정
    image_path = os.path.join(image_dir, image_file)
    label_path = os.path.join(label_dir, os.path.splitext(image_file)[0] + '.txt')
    
    # 디버깅 출력: 경로 확인
    print(f"Processing image: {image_path}")
    print(f"Corresponding label: {label_path}")
    
    # 이미지 로드
    image = cv2.imread(image_path)
    if image is None:
        print(f"Skipping {image_file}, unable to load image.")
        continue
    
    # 라벨링 파일 로드
    if os.path.exists(label_path):
        with open(label_path, 'r') as f:
            yolo_labels = f.readlines()
    else:
        print(f"Skipping {image_file}, corresponding label file not found.")
        continue

    # 바운딩 박스를 그린 이미지 생성
    visualized_image = draw_bounding_boxes(image, yolo_labels)

    # 이미지 저장
    output_image_path = os.path.join(output_image_dir, image_file)
    try:
        cv2.imwrite(output_image_path, visualized_image)
        print(f"Saved visualized image to: {output_image_path}")
    except Exception as e:
        print(f"Error saving image {output_image_path}: {e}")
    
    print(f"Processed {image_file} -> {output_image_path}")

print("모든 이미지에 라벨이 시각화되었습니다.")