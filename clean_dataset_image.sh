#!/bin/bash
# 일단 레이블과 쌍이 안맞는 이미지만 체크해서 제거 (전체 이미지 데이터셋에 대해 수행해야함)
for img in $1/*.json
do
kitti_name=$2/$(basename $img .json).jpg
if [  ! -f  $kitti_name ]; then
echo "$kitti_name not exist, will remove that image pair."
rm $img
fi
done

echo "$1: $(ls $1|wc -l)"
echo "$2: $(ls $2|wc -l)"
echo Done!
