index=1
for file in $(adb shell ls -tr /sdcard/FIRST/purePursuit_logs/*.csv); do
	adb pull "$file" ./"$index".csv
	index=$(expr $index + 1)
done
