import cv2
import os

def resize_and_pad(image, target_size):
    old_size = image.shape[:2]  # old_size is in (height, width) format
    ratio = float(target_size) / max(old_size)
    new_size = tuple([int(x * ratio) for x in old_size])

    # Resize the image
    resized_image = cv2.resize(image, (new_size[1], new_size[0]))

    # Create a new image of the target size and fill it with black
    delta_w = target_size - new_size[1]
    delta_h = target_size - new_size[0]
    top, bottom = delta_h // 2, delta_h - (delta_h // 2)
    left, right = delta_w // 2, delta_w - (delta_w // 2)

    new_image = cv2.copyMakeBorder(resized_image, 
                                   top, bottom, left, right, 
                                   cv2.BORDER_CONSTANT, value=[0, 0, 0])
    return new_image, ratio, new_size, top, left

def adjust_labels(yolo_labels, ratio, new_size, target_size, top, left):
    adjusted_labels = []
    for label in yolo_labels:
        cls, x_center, y_center, width, height = map(float, label.split())

        # Update center coordinates and dimensions to the resized image dimensions
        x_center = x_center * new_size[1]
        y_center = y_center * new_size[0]
        width = width * new_size[1]
        height = height * new_size[0]

        # Apply padding offsets
        x_center += left
        y_center += top

        # Normalize back to the target size
        x_center /= target_size
        y_center /= target_size
        width /= target_size
        height /= target_size

        adjusted_label = f"{cls} {x_center} {y_center} {width} {height}"
        adjusted_labels.append(adjusted_label)

        # 디버깅 출력
        print(f"Original label: {label}")
        print(f"Adjusted label: {adjusted_label}")

    return adjusted_labels

# 경로 설정
image_dir = '/Users/ijonghyeon/Downloads/생활 폐기물 이미지/images/val'
label_dir = '/Users/ijonghyeon/Downloads/생활 폐기물 이미지/labels/val'
output_image_dir = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/datasets/images/val'
output_label_dir = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/datasets/labels/val'

# 새로운 크기 설정 (예: 640x640)
target_size = 640

# 디렉토리가 존재하지 않으면 생성
if not os.path.exists(output_image_dir):
    os.makedirs(output_image_dir)
if not os.path.exists(output_label_dir):
    os.makedirs(output_label_dir)

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

    # 이미지 리사이즈 및 패딩
    resized_image, ratio, new_size, top, left = resize_and_pad(image, target_size)

    # 라벨 조정
    adjusted_labels = adjust_labels(yolo_labels, ratio, new_size, target_size, top, left)

    # 리사이즈된 이미지와 조정된 라벨 저장
    output_image_path = os.path.join(output_image_dir, image_file)
    output_label_path = os.path.join(output_label_dir, os.path.splitext(image_file)[0] + '.txt')

    try:
        cv2.imwrite(output_image_path, resized_image)
        print(f"Saved resized image to: {output_image_path}")
    except Exception as e:
        print(f"Error saving image {output_image_path}: {e}")

    try:
        with open(output_label_path, 'w') as f:
            f.write("\n".join(adjusted_labels))
        print(f"Saved adjusted labels to: {output_label_path}")
    except Exception as e:
        print(f"Error saving label {output_label_path}: {e}")
    
    print(f"Processed {image_file} -> {output_image_path}")

print("모든 이미지와 라벨링 정보가 성공적으로 리사이즈되고 패딩되었습니다.")