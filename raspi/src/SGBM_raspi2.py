import numpy as np
import cv2
import random
import time
import math
from sklearn.cluster import KMeans  
from sklearn.metrics import silhouette_score
import matplotlib.pyplot as plt
import cv2
import numpy as np
# import hough_detect as hd

def kmeans2(lines_k):#decline=inf#!scaling
    K=4
    kmeans = KMeans(n_clusters=K,n_init=10,init='k-means++',random_state=42)  

    # 对数据进行拟合和预测  
    kmeans.fit(lines_k)  
    # labels = kmeans.predict(lines_k)  
    centroids = kmeans.cluster_centers_  
    #! determine k,initial point
    # 打印聚类中心和标签  
    print("Cluster1 centers:")  
    print(centroids)  
    #[[-294.83887 1124.4237 ]
 #[-232.4528  1131.3954 ]]
    # # 可视化结果  
    # plt.scatter(lines_k[:, 0], lines_k[:, 1], c=labels, cmap='viridis')  
    # plt.scatter(centroids[:, 0], centroids[:, 1], c='red', s=300, alpha=0.5)  
    # plt.title('K-means Clustering')  
    # plt.xlabel('Feature 1')  
    # plt.ylabel('Feature 2')  
    # plt.show()
    K=2
    kmeans = KMeans(n_clusters=K,n_init=10,init='k-means++',random_state=42)  

    # 对数据进行拟合和预测  
    kmeans.fit(centroids)  
    # labels = kmeans.predict(centroids)  
    centroids2 = kmeans.cluster_centers_  
    #! determine k,initial point
    # 打印聚类中心和标签  
    print("Cluster2 centers:")  
    print(centroids2)
    #[[-294.83887 1124.4237 ]
    #[-232.4528  1131.3954 ]]
    # # 可视化结果  
    # plt.scatter(centroids[:, 0], centroids[:, 1], c=labels, cmap='viridis')  
    # plt.scatter(centroids2[:, 0], centroids2[:, 1], c='red', s=300, alpha=0.5)  
    # plt.title('K-means Clustering2')  
    # plt.xlabel('Feature 1')  
    # plt.ylabel('Feature 2')  
    # plt.show()
    if abs(centroids2[0][0]-centroids2[1][0])>100 or abs(centroids2[0][1]-centroids2[1][1])>100:
        if abs(centroids2[0][0])+abs(centroids2[0][1])<abs(centroids2[1][0])+abs(centroids2[1][1]):
            return [centroids2[0][0],centroids2[0][1]]
        else:
            return [centroids2[1][0],centroids2[1][1]]
    else:
        return [(centroids2[0][0]+centroids2[1][0])/2,(centroids2[0][1]+centroids2[1][1])/2]


def find_yellow_boxes(image):#!ignore small area

    yellowlow = np.array([0, 100, 120])
    yellowhigh = np.array([70, 255, 255])
    hsvimg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #将BGR色彩空间转化为HSV色彩空间 
    yellowmask = cv2.inRange(hsvimg, yellowlow, yellowhigh) #提取掩膜 
    yellowarea = cv2.bitwise_and(image, image, mask = yellowmask) #利用掩膜生成带有颜色的图像
    gray_image = cv2.cvtColor(yellowarea, cv2.COLOR_BGR2GRAY)
    # 使用阈值将灰度图像转换为二值图像
    _, binary_image = cv2.threshold(gray_image, 1, 255, cv2.THRESH_BINARY)
    # cv2.imshow("mask1", yellowarea)
    # cv2.waitKey(0)
    # cv2.imshow("mask2", binary_image)
    # cv2.waitKey(0)
        # 使用阈值分割将黄色区域与其他颜色分离
        # lower_yellow = np.array([70, 118, 180])
        # cv2.imshow("mask", mask)

        # 腐蚀和膨胀操作使得黄色区域变得更加连续
        
        # kernel = np.ones((5, 5), np.uint8)
    # mask = cv2.erode(mask, kernel, iterations=1)
    # mask = cv2.dilate(mask, kernel, iterations=1)


    # 寻找黄色区域的连通域
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_image, connectivity=8)

    # 为每个连通域绘制矩形框并返回矩形框参数
    boxes = []

    for i in range(0, num_labels):
        x, y, w, h, area = stats[i]
        if area > 400:  # 过滤掉面积过小的区域
            boxes.append([x, y, w, h])
            # cv2.namedWindow("yellow_boxes",0)
            # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # cv2.imshow("yellow_boxes", image)
            # cv2.waitKey(0)
            # cv2.rectangle(image2, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # cv2.imshow("yellow_boxes", image2)
            # cv2.waitKey(0)
    # cv2.destroyWindow("yellow_boxes")
    print("找到的黄色矩形框：", boxes)
    return boxes

def cal_average_distance_in_boxes(boxes,threeD):
    distance_list = []
    target_loc=[]
    for box in boxes:
        if box==[0,0,1280,720]:
            continue
        x= int(box[0])
        y= int(box[1])        
        w= int(box[2])
        h= int(box[3])
        
        for i in range(y,y+h):
            for j in range(x,x+w):
                if threeD[i][j][0]<4000 and threeD[i][j][1]<4000 and threeD[i][j][2]<4000:
                    distance_list.append([threeD[i][j][0],threeD[i][j][2]])
        if len(distance_list)>10:
            target_loc.append(kmeans2(np.array(distance_list)))
        distance_list=[]
    index_to_remove = []
    new_points = []  # 用于存储新生成的点

    for i in range(len(target_loc)):
        for j in range(i + 1, len(target_loc)):
            if abs(target_loc[i][0] - target_loc[j][0]) < 100 and abs(target_loc[i][1] - target_loc[j][1]) < 100:
                new_point = [(target_loc[i][0] + target_loc[j][0]) / 2, (target_loc[i][1] + target_loc[j][1]) / 2]
                new_points.append(new_point)  # 将新点记录在新列表中
                index_to_remove.extend([i, j])
                # print(f"合并{i}和{j}，新点{new_point}")
                break  # 找到一对后跳出内层循环，避免重复合并

    # 去除要删除的点
    index_to_remove = list(set(index_to_remove))  # 消除重复的索引
    for index in sorted(index_to_remove, reverse=True):
        del target_loc[index]

    # 将新生成的点添加到target_loc中
    target_loc.extend(new_points)

    # 去除要删除的点
    index_to_remove = list(set(index_to_remove))  # 消除重复的索引
    for index in sorted(index_to_remove, reverse=True):
        del target_loc[index]

    return target_loc

def find_target_loc():
    left_camera_matrix = np.array([[6.658779254398679e+02,0,6.456685972506924e+02],
    [0,6.654680419316929e+02,3.339317140939414e+02],
                                [0,0,1]])
    # left_camera_matrix = np.array([[516.5066236,-1.444673028,320.2950423],[0,516.5816117,270.7881873],[0.,0.,1.]])
    right_camera_matrix = np.array([[6.666340134907508e+02,0,6.450309322183250e+02],
    [0,6.662070938786943e+02,3.525276587772178e+02],
    [0,0,1]])

    left_distortion = np.array([[-0.053390802129390,0.127270321741189,5.196948472955012e-04,2.124273104283209e-04,-0.091559470679566]])
    right_distortion = np.array([[-0.060693317140140,0.148720908337062,-4.696798593761847e-04,-3.517967377954150e-05,-0.109112699405528]])

    R = np.array([[0.999380857975134,-0.030387164723515,0.017734738028798],
                [0.029695015211065,0.998833892144457,0.038066546667239],
                [-0.018870791835499,-0.037516344752927,0.999117819424657]])

    T = np.array([1.501071158968365e+02,4.603268774394913,-3.230774963979015])

    cap = cv2.VideoCapture("/dev/right_video") #r
    cap2 = cv2.VideoCapture("/dev/left_video")#l
# fps = 0.0
    width=1280
    height=720
    cap.set(3, width)  # 设置宽度为640
    cap.set(4, height)  # 设置高度为480
    cap2.set(3, width)  # 设置宽度为640
    cap2.set(4, height)  # 设置高度为480
    # cap.set(cv2.CAP_PROP_EXPOSURE, 20)  #设置曝光值 1.0 - 5000  156.0
    # cap2.set(cv2.CAP_PROP_EXPOSURE, 20)  #设置曝光值 1.0 - 5000  156.0
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    ret, frame1 = cap.read()#r
    ret2, frame2 = cap2.read()#l

    size=(width,height)
    # size = (640, 480)

    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                    right_camera_matrix, right_distortion, size, R,
                                                                    T)

    # 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
    left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)

    blockSize = 5
    img_channels = 3
    stereo = cv2.StereoSGBM_create(minDisparity=0,
                                numDisparities=128,
                                blockSize=blockSize,
                                P1=8 * img_channels * blockSize * blockSize,
                                P2=32 * img_channels * blockSize * blockSize,
                                disp12MaxDiff=-1,
                                preFilterCap=63,
                                uniquenessRatio=10,
                                speckleWindowSize=100,
                                speckleRange=1,
                                mode=cv2.STEREO_SGBM_MODE_HH)
    while cap.isOpened() and cap2.isOpened():
        ret, frame1 = cap.read()
        ret2, frame2 = cap2.read()
        if (ret and ret2):
            
            imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
            img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)
            # img1_rectified = imgL
            imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
            imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)
# --------------------------------------------------------------------------------------------
            # 计算视差
            disparity = stereo.compute(img1_rectified, img2_rectified)

            # 归一化函数算法，生成深度图（灰度图）
            disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # 生成深度图（颜色图）
            dis_color = disparity
            dis_color = cv2.normalize(dis_color, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            dis_color = cv2.applyColorMap(dis_color, 2)

            # 计算三维坐标数据值
            threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
            # 计算出的threeD，需要乘以16，才等于现实中的距离
            threeD = threeD * 16
            boxes=find_yellow_boxes(frame2)
            return cal_average_distance_in_boxes(boxes,threeD)
            
        else:
            print("No frame!")
            cap.release()
            cap2.release()
            cap = cv2.VideoCapture("/dev/r_video") #r
            cap2 = cv2.VideoCapture("/dev/l_video")#l
        # 若键盘按下q则退出播放
    # 释放资源

if __name__ == '__main__':
    target_loc=find_target_loc()
    print("target_loc:",target_loc)
    for i in target_loc:
        for j in target_loc:
            if i!=j and abs(i[0]-j[0])<100 and abs(i[1]-j[1])<100:
                target_loc.remove(j)
    # 关闭所有窗口
    # cv2.destroyAllWindows()
