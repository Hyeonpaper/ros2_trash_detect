import os
import json

# 원본 폴더와 대상 폴더 경로 설정
source_folder = "/Users/ijonghyeon/Downloads/생활 폐기물 이미지/labels/train_json"  # JSON 파일들이 있는 폴더 경로
destination_folder = "/Users/ijonghyeon/Downloads/생활 폐기물 이미지/labels/train"  # YOLO 형식 파일을 저장할 폴더 경로

# 대상 폴더가 존재하지 않으면 생성
os.makedirs(destination_folder, exist_ok=True)

# 클래스와 클래스 ID 매핑
class_mapping = {
    "비닐류": 0,
    "페트병류": 1,
    "종이류": 2,
    "캔류": 3,
    "유리병류": 4
}

# 바운딩 박스를 YOLO 형식으로 변환하는 함수
def convert_to_yolo_format(bbox, img_width, img_height):
    try:
        x1, y1, x2, y2 = int(bbox['x1']), int(bbox['y1']), int(bbox['x2']), int(bbox['y2'])
        x_center = ((x1 + x2) / 2) / img_width
        y_center = ((y1 + y2) / 2) / img_height
        width = (x2 - x1) / img_width
        height = (y2 - y1) / img_height
        return x_center, y_center, width, height
    except Exception as e:
        print(f"Error converting bounding box: {bbox}, error: {e}")
        return None

# JSON 파일을 처리하는 함수
def process_json_file(file_path):
    try:
        print(f"Processing file: {file_path}")  # 디버깅을 위한 출력
        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
        print(f"File loaded: {file_path}")  # 파일 로드 확인

        if 'RESOLUTION' not in data:
            print(f"Missing 'RESOLUTION' in file {file_path}")
            return

        img_width, img_height = map(int, data['RESOLUTION'].split('*'))
        print(f"Image dimensions: width={img_width}, height={img_height}")

        yolo_data = []

        if 'Bounding' not in data:
            print(f"Missing 'Bounding' in file {file_path}")
            return

        for bounding in data['Bounding']:
            if 'CLASS' not in bounding:
                print(f"Missing 'CLASS' in bounding box: {bounding}")
                continue
            class_name = bounding['CLASS']  # 클래스 이름 가져오기
            class_id = class_mapping.get(class_name, -1)  # 클래스 ID 매핑
            if class_id == -1:
                print(f"Unknown class: {class_name}, skipping...")
                continue
            bbox = convert_to_yolo_format(bounding, img_width, img_height)
            if bbox is not None:
                print(f"Converted bounding box: {bbox}")
                yolo_data.append(f"{class_id} {' '.join(map(str, bbox))}")
            else:
                print(f"Skipping bounding box: {bounding}")

        # YOLO 형식 파일로 저장
        if yolo_data:  # yolo_data가 비어있지 않은 경우에만 파일 저장
            yolo_filename = os.path.splitext(os.path.basename(file_path))[0] + ".txt"
            yolo_filepath = os.path.join(destination_folder, yolo_filename)
            print(f"Saving YOLO format to: {yolo_filepath}")  # 디버깅을 위한 출력
            with open(yolo_filepath, 'w', encoding='utf-8') as yolo_file:
                yolo_file.write("\n".join(yolo_data))
            print(f"File saved: {yolo_filepath}")
        else:
            print(f"No bounding boxes to save for file: {file_path}")
    except Exception as e:
        print(f"Failed to process file {file_path}: {e}")  # 에러 발생 시 출력

# 원본 폴더의 모든 JSON 파일을 처리
try:
    print("Listing files in source folder...")  # 디버깅을 위한 출력
    files = os.listdir(source_folder)
    print(f"Files found: {files}")  # 폴더 내 파일 목록 출력
    for filename in files:
        file_path = os.path.join(source_folder, filename)
        if filename.lower().endswith(".json"):  # 확장자를 소문자로 변환하여 체크
            print(f"Found JSON file: {file_path}")
            process_json_file(file_path)
        else:
            print(f"Skipping non-JSON file: {filename}")
except Exception as e:
    print(f"Error listing files in source folder: {e}")  # 에러 발생 시 출력

print("모든 JSON 파일을 YOLO 형식으로 변환하였습니다.")
