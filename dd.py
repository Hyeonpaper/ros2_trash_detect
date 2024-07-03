import os

def get_files_without_extension(directory):
    files = os.listdir(directory)
    file_names = [os.path.splitext(file)[0] for file in files]
    return set(file_names), files

def delete_unpaired_files(dir1, dir2):
    dir1_names, dir1_files = get_files_without_extension(dir1)
    dir2_names, dir2_files = get_files_without_extension(dir2)

    # Find unpaired files in dir1
    unpaired_in_dir1 = dir1_names - dir2_names
    # Find unpaired files in dir2
    unpaired_in_dir2 = dir2_names - dir1_names

    # Delete unpaired files in dir1
    for file in dir1_files:
        if os.path.splitext(file)[0] in unpaired_in_dir1:
            os.remove(os.path.join(dir1, file))
            print(f"Deleted {file} from {dir1}")

    # Delete unpaired files in dir2
    for file in dir2_files:
        if os.path.splitext(file)[0] in unpaired_in_dir2:
            os.remove(os.path.join(dir2, file))
            print(f"Deleted {file} from {dir2}")

# Usage example:
directory1 = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/datasets/split_images/train/train'
directory2 = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/datasets/split_labels/train/train'

delete_unpaired_files(directory1, directory2)