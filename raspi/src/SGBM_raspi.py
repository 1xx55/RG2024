import numpy as np
import cv2
import random

# import time
# import math

#from sklearn.cluster import KMeans  
# from sklearn.metrics import silhouette_score
# import matplotlib.pyplot as plt
# import cv2
# import numpy as np
# import hough_detect as hd
x_error=15
y_error=10
# def kmeans2(lines_k):#decline=inf#!scaling
#     K=4
#     kmeans = KMeans(n_clusters=K,n_init=10,init='k-means++',random_state=42)  

#     # 对数据进行拟合和预测  
#     kmeans.fit(lines_k)  
#     # labels = kmeans.predict(lines_k)  
#     centroids = kmeans.cluster_centers_  
#     #! determine k,initial point
#     # 打印聚类中心和标签  
#     #[[-294.83887 1124.4237 ]
#  #[-232.4528  1131.3954 ]]
#     # # 可视化结果  
#     # plt.scatter(lines_k[:, 0], lines_k[:, 1], c=labels, cmap='viridis')  
#     # plt.scatter(centroids[:, 0], centroids[:, 1], c='red', s=300, alpha=0.5)  
#     # plt.title('K-means Clustering')  
#     # plt.xlabel('Feature 1')  
#     # plt.ylabel('Feature 2')  
#     # plt.show()
#     K=2
#     kmeans = KMeans(n_clusters=K,n_init=10,init='k-means++',random_state=42)  

#     # 对数据进行拟合和预测  
#     kmeans.fit(centroids)  
#     # labels = kmeans.predict(centroids)  
#     centroids2 = kmeans.cluster_centers_  
#     print("Cluster2 centers:",centroids2)
#     #! determine k,initial point
#     # 打印聚类中心和标签  
#     #print("Cluster2 centers:")  
#     #print(centroids2)
#     #[[-294.83887 1124.4237 ]
#     #[-232.4528  1131.3954 ]]
#     # # 可视化结果  
#     # plt.scatter(centroids[:, 0], centroids[:, 1], c=labels, cmap='viridis')  
#     # plt.scatter(centroids2[:, 0], centroids2[:, 1], c='red', s=300, alpha=0.5)  
#     # plt.title('K-means Clustering2')  
#     # plt.xlabel('Feature 1')  
#     # plt.ylabel('Feature 2')  
#     # plt.show()
#     if abs(centroids2[0][0]-centroids2[1][0])>100 or abs(centroids2[0][1]-centroids2[1][1])>100:
#         if abs(centroids2[0][0])+abs(centroids2[0][1])<abs(centroids2[1][0])+abs(centroids2[1][1]):
#             return [centroids2[0][0],centroids2[0][1]]
#         else:
#             return [centroids2[1][0],centroids2[1][1]]
#     else:
#         return [(centroids2[0][0]+centroids2[1][0])/2,(centroids2[0][1]+centroids2[1][1])/2]

def kmeans(X, K, max_iters=100):
    # 随机选择K个初始簇中心
    centroids = X[np.random.choice(X.shape[0], K, replace=False)]

    for _ in range(max_iters):
        # 计算每个数据点到簇中心的距离
        distances = np.linalg.norm(X[:, np.newaxis] - centroids, axis=2)

        # 将数据点分配到最近的簇中心
        labels = np.argmin(distances, axis=1)
        new_centroids = []
        for k in range(K):
            cluster_data = X[labels == k]
            if len(cluster_data) > 0:
                new_centroid = cluster_data.mean(axis=0)
            else:
                # 如果聚类为空，设置一个默认质心
                new_centroid = np.array([0, 0])
            new_centroids.append(new_centroid)
        new_centroids = np.array(new_centroids)
        # 更新簇中心为簇内所有点的均值
        # new_centroids = np.array([X[labels == k].mean(axis=0) for k in range(K)])

        # 如果簇中心没有变化，停止迭代
        if np.all(centroids == new_centroids):
            print("Converged at iteration {}".format(_))
            break

        centroids = new_centroids

    return centroids, labels
def Kmeans2(lines_k):
    K=2
# 调用kmeans函数
    centroids, labels = kmeans(lines_k, K)
    cluster_counts = np.bincount(labels)
    max_cluster_index = np.argmax(cluster_counts)
    max_cluster_center = centroids[max_cluster_index]
    return max_cluster_center
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
            boxes.append([x+x_error, y+y_error, min(w,1280-x-x_error), min(h,720-y-y_error)])
    #         cv2.namedWindow("yellow_boxes",0)
    #         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #         cv2.imshow("yellow_boxes", image)
    #         cv2.waitKey(0)
    #         cv2.rectangle(image2, (x+x_error, y+y_error), (x + w+x_error, y+y_error + h), (0, 255, 0), 2)
    #         cv2.imshow("yellow_boxes", image2)
    #         cv2.waitKey(0)
    # cv2.destroyWindow("yellow_boxes")
    # print("找到的黄色矩形框：", boxes)
    return boxes

def cal_average_distance_in_boxes(boxes,threeD,theta):
    distance_list = []
    target_loc=[]
    for box in boxes:
        if box==[x_error,y_error,1280-x_error,720-y_error]:
            continue
        x= int(box[0])
        y= int(box[1])        
        w= int(box[2])
        h= int(box[3])
        
        for i in range(y,y+h-1):
            for j in range(x,x+w-1):
                if abs(threeD[i][j][0])<2000 and abs(threeD[i][j][1])<2000 and abs(threeD[i][j][2])<2000:
                    distance_list.append([threeD[i][j][0],threeD[i][j][2]])
        if len(distance_list) > 40000:
            sample_size = len(distance_list) // 10
            distance_list = random.sample(distance_list, sample_size)
            target_loc.append(Kmeans2(np.array(distance_list)))
        elif len(distance_list)>10:
            target_loc.append(Kmeans2(np.array(distance_list)))
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
    # print("目标点：", target_loc)
    # 去除要删除的点
    target_loc=[[int(i[0]),int(i[1]*theta)] for i in target_loc]
    print(target_loc)
    return target_loc


def find_target_loc():

    cap = cv2.VideoCapture(0) #r
    cap2 = cv2.VideoCapture(2)#l
# fps = 0.0
    width=1280
    height=720
    cap.set(3, width)  # 设置宽度为640
    cap.set(4, height)  # 设置高度为480
    cap2.set(3, width)  # 设置宽度为640
    cap2.set(4, height)  # 设置高度为480
    cap.set(cv2.CAP_PROP_EXPOSURE, -2)  #设置曝光值 1.0 - 5000  156.0
    cap2.set(cv2.CAP_PROP_EXPOSURE, -2)  #设置曝光值 1.0 - 5000  156.0
    ret, frame1 = cap.read()#r
    ret2, frame2 = cap2.read()#l
    # battery_left = np.array([[6.643016300004764e+02,0,6.563100071373611e+02],
    # [0,6.644247477622929e+02,3.556354805108356e+02],
    # [0,0,1]])
        
    # battery_right = np.array([[6.673246357778347e+02,0,6.772090131665929e+02],
    # [0,6.662543831670702e+02,3.424851211099173e+02],
    # [0,0,1]])

    # battery_left_distortion = np.array([[-0.074904357204369,0.281864841179994,-5.198081343035242e-04,2.168890356671976e-04,-0.269609744973071]])

    # battery_right_distortion = np.array([[-0.080629130777460,0.256741278909257,2.176717108150630e-04,8.864571839108397e-04,-0.219598807029986]])

    # # R = np.array([[0.999986459881979,-0.004822102418715,-0.001956369333716],
    # #               [0.004850026864318,0.999882682839651,0.014529204960947],
    # #     [0.001886078503637,-0.014538496677622,0.999892531636392]])
    # R = np.array([[0.999978786379603,-0.004560673244851,-0.004650489257011],
    # [0.004647936384322,0.999810019168235,0.018929401949888],
    # [0.004563274936180,-0.018950615566964,0.999810007297131]])
    # T = np.array([-52.856130300655120,0.062397897038920,-0.827713154403646])

    # size=(1280,720)
    # R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(battery_left, battery_left_distortion,
    #                                                                 battery_right, battery_right_distortion, size, R,
    #                                                                 T)
    # left_map1, left_map2 = cv2.initUndistortRectifyMap(battery_left, battery_left_distortion, R1, P1, size, cv2.CV_16SC2)
    # right_map1, right_map2 = cv2.initUndistortRectifyMap(battery_right, battery_right_distortion, R2, P2, size, cv2.CV_16SC2)
    # blockSize = 5
    # img_channels = 3
    # stereo = cv2.StereoSGBM_create(minDisparity=0,
    #                             numDisparities=128,
    #                             blockSize=blockSize,
    #                             P1=8 * img_channels * blockSize * blockSize,
    #                             P2=32 * img_channels * blockSize * blockSize,
    #                             disp12MaxDiff=-1,
    #                             preFilterCap=63,
    #                             uniquenessRatio=10,
    #                             speckleWindowSize=100,
    #                             speckleRange=1,
    #                             mode=cv2.STEREO_SGBM_MODE_HH)
    # pump_left = np.array([[6.629714219465708e+02,0,6.552764493603507e+02],
    #                       [0,6.627627401785837e+02,3.673866482137107e+02],[0,0,1]])
        
    # pump_right = np.array([[6.630597501042919e+02,0,6.470821199204139e+02],
    # [0,6.626336469394710e+02,3.314077988629549e+02],
    # [0,0,1]])

    # pump_left_distortion = np.array([[-0.069666500520591,0.210081077069142,-1.857502891772009e-05,1.736149289216960e-04,-0.183842295110516]])

    # pump_right_distortion = np.array([[-0.071516663523896,0.223341775166155,-1.832639845268067e-04,-0.001159739562814,-0.212903780057474]])

    # R = np.array([[0.999970505713521,-7.602754114721856e-04,0.007642622870676],
    # [6.621159550494266e-04,0.999917369978004,0.012838022384029],
    #               [-0.007651751793331,-0.012832583433178,0.999888381519119]])

    # T = np.array([-54.270925850730485,0.044042542371004,0.928487068566897])

    # size=(1280,720)
    # R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(pump_left, pump_left_distortion,
    #                                                                 pump_right, pump_right_distortion, size, R,
    #                                                                 T)
    # left_map1, left_map2 = cv2.initUndistortRectifyMap(pump_left, pump_left_distortion, R1, P1, size, cv2.CV_16SC2)
    # right_map1, right_map2 = cv2.initUndistortRectifyMap(pump_right, pump_right_distortion, R2, P2, size, cv2.CV_16SC2)
    # blockSize = 5
    # img_channels = 3
    # stereo = cv2.StereoSGBM_create(minDisparity=0,
    #                             numDisparities=128,
    #                             blockSize=blockSize,
    #                             P1=8 * img_channels * blockSize * blockSize,
    #                             P2=32 * img_channels * blockSize * blockSize,
    #                             disp12MaxDiff=-1,
    #                             preFilterCap=63,
    #                             uniquenessRatio=10,
    #                             speckleWindowSize=100,
    #                             speckleRange=1,
    #                             mode=cv2.STEREO_SGBM_MODE_HH)
    back_left = np.array([[6.619429562862643e+02,0,6.459605492240485e+02],
    [0,6.614350932769422e+02,3.525891388907261e+02],
    [0,0,1]])
        
    back_right = np.array([[6.605128124705367e+02,0,6.459880562504392e+02],
    [0,6.603348282504245e+02,3.339124638096636e+02],
    [0,0,1]])

    back_left_distortion = np.array([[-0.072140480581682,0.192434356784887,6.729836035210499e-05,-3.089012912920528e-05,-0.164200629150463]])

    back_right_distortion = np.array([[-0.077506564166491,0.243546646139244,4.979600291308596e-04,-1.101534692040529e-04,-0.268708671122290]])

    R = np.array([[0.998742607316517,0.016406905687746,-0.047371064758555],
    [-0.015400883773296,0.999649692559833,0.021524519600508],
    [0.047707621085181,-0.020767898564483,0.998645421197939]])

    T = np.array([-81.656461797819500,1.064811399327354,-0.786899592962507])

    size=(1280,720)
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(back_left, back_left_distortion,
                                                                    back_right, back_right_distortion, size, R,
                                                                    T)
    left_map1, left_map2 = cv2.initUndistortRectifyMap(back_left, back_left_distortion, R1, P1, size, cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(back_right, back_right_distortion, R2, P2, size, cv2.CV_16SC2)
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
            cv2.imshow('disparity', dis_color)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                boxes=find_yellow_boxes(frame1,dis_color)
                return cal_average_distance_in_boxes(boxes,threeD)
            
        else:
            print("No frame!")
            cap.release()
            cap2.release()
            cap = cv2.VideoCapture(0) #r
            cap2 = cv2.VideoCapture(2)#l
        # 若键盘按下q则退出播放
    # 释放资源

if __name__ == '__main__':
    target_loc=find_target_loc()
    print("target_loc:",target_loc)
    # 关闭所有窗口
    # cv2.destroyAllWindows()
