import os

def count_files_with_prefixes(directory, prefixes):
    counts = {prefix: 0 for prefix in prefixes}

    for filename in os.listdir(directory):
        for prefix in prefixes:
            if filename.startswith(str(prefix)):
                counts[prefix] += 1
                break

    return counts

# 경로 설정
directory = '/Users/ijonghyeon/Desktop/대학/전공/3학년/로봇프로그래밍/프로젝트/datasets/images/val28'
prefixes = [15, 17, 21, 22, 23]

# 파일 개수 세기
counts = count_files_with_prefixes(directory, prefixes)

# 결과 출력
for prefix, count in counts.items():
    print(f"Files starting with {prefix}: {count}")