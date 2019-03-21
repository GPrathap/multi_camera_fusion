#!/bin/bash
BAG_DIR=../data/bag
SERVER_HOST=dgx
SERVER_USER=kiasoul
SERVER_FOLDER=/home/datasets/kia

for f in $(ls $BAG_DIR/*.bag* | xargs -n 1 basename)
do
	IFS='-' read -r -a array <<< "$f"
	SUBFOLDER=${array[2]}-${array[1]}-${array[0]}
	echo "Uploading file $f to $SERVER_HOST:$SERVER_FOLDER/$SUBFOLDER/$f"
	ssh $SERVER_USER@$SERVER_HOST -t "mkdir $SERVER_FOLDER/$SUBFOLDER"		
	scp $BAG_DIR/$f $SERVER_USER@$SERVER_HOST:$SERVER_FOLDER/$SUBFOLDER/$f && rm $BAG_DIR/$f
done
