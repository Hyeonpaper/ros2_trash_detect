import os
import shutil
import random

# 원본 이미지 및 라벨 파일 경로 설정
source_image_folder = "/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/image"  # 이미지 파일들이 있는 폴더 경로
source_label_folder = "/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/label_txt"  # YOLO 형식 라벨 파일들이 있는 폴더 경로
destination_folder = "/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/dataset"  # YOLO 형식 데이터셋을 저장할 폴더 경로

# 폴더 구조 생성
os.makedirs(os.path.join(destination_folder, "images/train"), exist_ok=True)
os.makedirs(os.path.join(destination_folder, "images/val"), exist_ok=True)
os.makedirs(os.path.join(destination_folder, "labels/train"), exist_ok=True)
os.makedirs(os.path.join(destination_folder, "labels/val"), exist_ok=True)

# 파일을 학습 및 검증 세트로 나누는 함수
def split_data(files, train_ratio=0.8):
    random.shuffle(files)
    train_files = files[:int(len(files) * train_ratio)]
    val_files = files[int(len(files) * train_ratio):]
    return train_files, val_files

# 파일 리스트 가져오기
image_files = [f for f in os.listdir(source_image_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
label_files = [f for f in os.listdir(source_label_folder) if f.lower().endswith('.txt')]

# 이미지 파일과 라벨 파일이 일치하는지 확인
image_files_base = {os.path.splitext(f)[0] for f in image_files}
label_files_base = {os.path.splitext(f)[0] for f in label_files}
common_files = list(image_files_base & label_files_base)

# 학습 및 검증 파일 리스트 생성
train_files, val_files = split_data(common_files)

# 데이터셋 준비 함수
def prepare_dataset(file_list, split):
    for base_filename in file_list:
        image_filename = base_filename + '.jpg'  # 또는 .png, .jpeg
        label_filename = base_filename + '.txt'

        if not os.path.exists(os.path.join(source_image_folder, image_filename)) or not os.path.exists(os.path.join(source_label_folder, label_filename)):
            print(f"Missing image or label file for base name: {base_filename}, skipping...")
            continue

        # 이미지 파일 복사
        if split == "train":
            shutil.copy(os.path.join(source_image_folder, image_filename), os.path.join(destination_folder, "images/train", image_filename))
            shutil.copy(os.path.join(source_label_folder, label_filename), os.path.join(destination_folder, "labels/train", label_filename))
        else:
            shutil.copy(os.path.join(source_image_folder, image_filename), os.path.join(destination_folder, "images/val", image_filename))
            shutil.copy(os.path.join(source_label_folder, label_filename), os.path.join(destination_folder, "labels/val", label_filename))

# 학습 및 검증 데이터셋 준비
prepare_dataset(train_files, "train")
prepare_dataset(val_files, "val")

print("Dataset preparation is complete.")
