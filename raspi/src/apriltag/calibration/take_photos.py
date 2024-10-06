import cv2
import os

# 设置保存照片的路径
save_path = '/Users/yao/Documents/Mingming LAB_intern/calibration'
if not os.path.exists(save_path):
    os.makedirs(save_path)

# 设置相机的ID号（0通常是默认相机）
camera_id = 0
cap = cv2.VideoCapture(camera_id)

# 设置照片编号
photo_number = 0

print("按 's' 键拍照，按 'q' 键退出。")

while True:
    # 捕获frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("无法获取画面，请检查相机连接。")
        break

    # 显示当前帧
    cv2.imshow('Camera', frame)

    # 等待键盘输入
    key = cv2.waitKey(1) & 0xFF

    # 如果按下's'，保存一张照片
    if key == ord('s'):
        photo_name = f'calibration_image_{photo_number}.jpg'
        photo_path = os.path.join(save_path, photo_name)
        cv2.imwrite(photo_path, frame)
        print(f'照片已保存到：{photo_path}')
        photo_number += 1

    # 如果按下'q'，退出循环
    elif key == ord('q'):
        break

# 释放摄像头并关闭所有OpenCV窗口
cap.release()
cv2.destroyAllWindows()
