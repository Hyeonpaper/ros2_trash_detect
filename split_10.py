import os
import shutil
import random

# 경로 설정
image_dir = '/Users/ijonghyeon/Desktop/dataset/images/val'
label_dir = '/Users/ijonghyeon/Desktop/dataset/labels/val'
output_base_image_dir = '/Users/ijonghyeon/Desktop/dataset/split_img/val'
output_base_label_dir = '/Users/ijonghyeon/Desktop/dataset/split_label/val'

# 분할할 폴더 수
num_folders = 5

# 디렉토리가 존재하지 않으면 생성
for i in range(1, num_folders + 1):
    image_folder = os.path.join(output_base_image_dir, f'images{i:02}')
    label_folder = os.path.join(output_base_label_dir, f'labels{i:02}')
    if not os.path.exists(image_folder):
        os.makedirs(image_folder)
    if not os.path.exists(label_folder):
        os.makedirs(label_folder)

# 이미지와 라벨 파일 리스트를 가져옴
image_files = sorted([f for f in os.listdir(image_dir) if f.endswith('.jpg') or f.endswith('.png')])
label_files = sorted([f for f in os.listdir(label_dir) if f.endswith('.txt')])

# 파일 수 확인
assert len(image_files) == len(label_files), "이미지와 라벨 파일의 수가 맞지 않습니다."

# 파일 쌍 만들기 (이름이 같은 것 끼리 매칭)
file_pairs = list(zip(image_files, label_files))

# 파일 쌍 섞기
random.shuffle(file_pairs)

# 분할 처리
for idx, (image_file, label_file) in enumerate(file_pairs):
    folder_idx = (idx % num_folders) + 1
    
    image_src = os.path.join(image_dir, image_file)
    label_src = os.path.join(label_dir, label_file)
    
    image_dst = os.path.join(output_base_image_dir, f'images{folder_idx:02}', image_file)
    label_dst = os.path.join(output_base_label_dir, f'labels{folder_idx:02}', label_file)
    
    shutil.copy(image_src, image_dst)
    shutil.copy(label_src, label_dst)
    
    if idx % 1000 == 0:
        print(f"{idx}개의 파일이 처리되었습니다.")

print("모든 파일이 성공적으로 랜덤하게 분할되었습니다.")