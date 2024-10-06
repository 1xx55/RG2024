import time
import cv2
import numpy as np
import math
from sklearn.cluster import KMeans  
from sklearn.metrics import silhouette_score
import matplotlib.pyplot as plt  
import policy
pi=math.pi
#todo take avarage of n points
def get_crosspoint(points,width,height):#!最高与最低连可能有问题
	min_value = min(sublist[1] for sublist in points)
	max_value = max(sublist[1] for sublist in points)
	max_index = [i for i, sublist in enumerate(points) if sublist[1] == max_value][0]
	min_index = [i for i, sublist in enumerate(points) if sublist[1] == min_value][0]

	k=(points[max_index][1]-points[min_index][1])/(points[max_index][0]-points[min_index][0])
	b=(points[max_index][1]-points[max_index][0]*k)
	line1=[k,b]
	#point1( points[max_index][0],)
	y=[None]*2
	x=[None]*2
	cnt=0
	for i in range(len(points)):
		if i!= max_index and i!=min_index:
			y[cnt]=points[i][1]
			x[cnt]=points[i][0]
			cnt=cnt+1
	k=(y[1]-y[0])/(x[1]-x[0])
	b=(y[1]-x[1]*k)
	line2=[k,b]
	return judge_cross(line1,line2,width,height)

def judge_cross(line1,line2,width,height):
    #line=[k,b]
    if line1[0]==line2[0]:
        return None
    x=(line1[1]-line2[1])/(line2[0]-line1[0])

    if x<=width+10 and x>=-10:
        y=line1[0]*x+line1[1]
        if y>=-10 and y<3*height/5+10:
            return [x,y]
    return None
def remove_none_lists(two_dim_list):
    return [lst for lst in two_dim_list if lst is not None]
def kmeans_points(lines_k):# points
#! 计算不同K值下的轮廓系数
	# range_Kmeans=range(2,6)
	# scores = []
	# for k in range_Kmeans:
	# 	kmeans = KMeans(n_clusters=k, random_state=42).fit(lines_k)
	# 	score = silhouette_score(lines_k, kmeans.labels_)
	# 	scores.append(score)

	# # 绘制轮廓系数图
	# plt.plot(range_Kmeans, scores, marker='o')
	# plt.xlabel('Number of clusters')
	# plt.ylabel('Silhouette Score')
	# plt.show()

	# # #!KMeans++
	# max_value = max(scores)
	# max_index = [i for i, j in enumerate(scores) if j == max_value][0]

	# K=max_index+2
	kmeans = KMeans(n_clusters=4, init='k-means++')  
	
	# 对数据进行拟合和预测  
	kmeans.fit(lines_k)  
	labels = kmeans.predict(lines_k)  
	centroids = kmeans.cluster_centers_  
	#! determine k,initial point
	# 打印聚类中心和标签  
	# print("Cluster centers:")  
	# print(centroids)  
	# print("Labels:")  
	# print(labels)  
	# for line in centroids:
	# 	cv2.line(img2, (0, int(line[1])), (width, int(line[0]*width+line[1])), (0, 255, 0), 2)
	return list(centroids)
	# cv2.imshow('edges', img2)
	# cv2.waitKey()
	# # 可视化结果  
	# plt.scatter(lines_k[:, 0], lines_k[:, 1], c=labels, cmap='viridis')  
	# plt.scatter(centroids[:, 0], centroids[:, 1], c='red', s=300, alpha=0.5)  
	# plt.title('K-means Clustering')  
	# plt.xlabel('Feature 1')  
	# plt.ylabel('Feature 2')  
	# plt.show()


#*autoscaling?
def kmeans1(lines_k,img2):# normal#!k,b didn't match after scaling
#! 计算不同K值下的轮廓系数
	height, width, channels = img2.shape
	#todo change_scale
	scale=100
	range_Kmeans=range(4,min(8,len(lines_k)))
	scores = []
	for k in range_Kmeans:
		kmeans = KMeans(n_clusters=k, random_state=42).fit(lines_k)
		score = silhouette_score(lines_k, kmeans.labels_)
		scores.append(score)

	# # # 绘制轮廓系数图
	# plt.plot(range_Kmeans, scores, marker='o')
	# plt.xlabel('Number of clusters')
	# plt.ylabel('Silhouette Score')
	# plt.show()

	# #!KMeans++
	if scores is None:
		return None
	max_value = max(scores)
	max_index = [i for i, j in enumerate(scores) if j == max_value][0]

	K=max_index+4
	kmeans = KMeans(n_clusters=K, init='k-means++')  
	
	# 对数据进行拟合和预测  
	kmeans.fit(lines_k)  
	labels = kmeans.predict(lines_k)  
	centroids = kmeans.cluster_centers_  
	#! determine k,initial point
	# 打印聚类中心和标签  
	# print("Cluster centers:")  
	# print(centroids)  
	# print("Labels:")  
	# print(labels)  
	 #! 可视化结果  
	# for line in centroids:
	# 	cv2.line(img2, (0, int(line[1]*scale)), (width, int(line[0]*width+line[1]*scale)), (0, 255, 0), 2)
	# cv2.imshow('edges', img2)
	# cv2.waitKey()
	#
	# plt.scatter(lines_k[:, 0], lines_k[:, 1], c=labels, cmap='viridis')  
	# plt.scatter(centroids[:, 0], centroids[:, 1], c='red', s=300, alpha=0.5)  
	# plt.title('K-means Clustering')  
	# plt.xlabel('Feature 1')  
	# plt.ylabel('Feature 2')  
	# plt.show()
	return list(centroids)
def kmeans2(lines_k,img2):#decline=inf#!scaling
#! 计算不同K值下的轮廓系数
	height, width, channels = img2.shape
	range_Kmeans=range(2,min(5,len(lines_k)))
	scores = []
	for k in range_Kmeans:
		kmeans = KMeans(n_clusters=k, random_state=42).fit(lines_k)
		score = silhouette_score(lines_k, kmeans.labels_)
		scores.append(score)

	# # 绘制轮廓系数图
	# plt.plot(range_Kmeans, scores, marker='o')
	# plt.xlabel('Number of clusters')
	# plt.ylabel('Silhouette Score')
	# plt.show()

	# #!KMeans++
	max_value = max(scores)
	max_index = [i for i, j in enumerate(scores) if j == max_value][0]

	K=max_index+2
	kmeans = KMeans(n_clusters=K, init='k-means++')  
	
	# 对数据进行拟合和预测  
	kmeans.fit(lines_k)  
	labels = kmeans.predict(lines_k)  
	centroids = kmeans.cluster_centers_  
	#! determine k,initial point
	# 打印聚类中心和标签  
	# print("Cluster centers:")  
	# print(centroids)  
	# print("Labels:")  
	# print(labels)  
	#! 可视化结果 
	# for line in centroids:
	# 	cv2.line(img2, (0, int(line[1]*1000)), (width, int(line[0]*width+line[1]*1000)), (0, 255, 0), 2)
	# cv2.imshow('edges', img2)
	# cv2.waitKey(0)
	# return list(centroids)
	
	#  
	# plt.scatter(lines_k[:, 0], lines_k[:, 1], c=labels, cmap='viridis')  
	# plt.scatter(centroids[:, 0], centroids[:, 1], c='red', s=300, alpha=0.5)  
	# plt.title('K-means Clustering')  
	# plt.xlabel('Feature 1')  
	# plt.ylabel('Feature 2')  
	# plt.show()

def switch_lines(lines):#!inf_/1000,normal_/100
	lines_k = [None] * len(lines)  # 创建一个与lines长度相同的列表，用None填充
	lines_j = [None] * len(lines)  # 创建一个与lines长度相同的列表，用None填充
	i=0
	j=0
	for line in lines:
		x1 = line[0][0]
		y1 = line[0][1]
		x2 = line[0][2]
		y2 = line[0][3]
		if x2 ==x1:
			k=6
		else:
			k=(y2-y1)/(x2-x1)
		if(k>1.7 or k<-1.7):
			b=(y1-x1*k)/1000
			lines_j[j]=[k,b]
			j=j+1
		else:
			b=(y1-x1*k)/100
			lines_k[i]=[k,b]
			i=i+1
	lines_array1 = np.array(remove_none_lists(lines_k))
	lines_array2 = np.array(remove_none_lists(lines_j))
	return (lines_array1,lines_array2)
		

def scale_elements(matrix,scale):
    result = []
    for row in matrix:
        new_row = [row[0],row[1]*scale]
        result.append(new_row)
    return result
def get_points(lines,width,height):
	#!get points
	length=len(lines)*len(lines)
	points=[None]*length
	cnt=0
	for i in range(len(lines)):
		for j in range(i+1,len(lines)):
			points[cnt]=judge_cross(lines[i],lines[j],width,height)
			cnt=cnt+1
	points=remove_none_lists(points)
	return points
def draw_point(points,img2):
	color = (0, 0, 255)  # 颜色为白色
	radius = 5  # 半径为1
	thickness = -1  # 线条粗细为-1，表示填充圆
	for point in points:
		cv2.circle(img2, (int(point[0]),int(point[1])), radius, color, thickness)
	# cv2.circle(img2, (int(points[0]),int(points[1])), radius, color, thickness)
	cv2.imshow('edges', img2)
	cv2.waitKey()
def switch_lines(lines):#!inf_/1000,normal_/100
	lines_k = [None] * len(lines)  # 创建一个与lines长度相同的列表，用None填充
	lines_j = [None] * len(lines)  # 创建一个与lines长度相同的列表，用None填充
	i=0
	j=0
	for line in lines:
		x1 = line[0][0]
		y1 = line[0][1]
		x2 = line[0][2]
		y2 = line[0][3]
		if x2 ==x1:
			k=6
		else:
			k=(y2-y1)/(x2-x1)
		if(k>1.7 or k<-1.7):
			b=(y1-x1*k)/1000
			lines_j[j]=[k,b]
			j=j+1
		else:
			b=(y1-x1*k)/100
			lines_k[i]=[k,b]
			i=i+1
	lines_array1 = np.array(remove_none_lists(lines_k))
	lines_array2 = np.array(remove_none_lists(lines_j))
	return (lines_array1,lines_array2)
import math


def calculate_angle(img):
    # img=img[0:300,:,:]
    yellowlow = np.array([0, 100, 100])
    yellowhigh = np.array([20, 255, 255])
    hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #将BGR色彩空间转化为HSV色彩空间 
    yellowmask = cv2.inRange(hsvimg, yellowlow, yellowhigh) #提取掩膜 
    yellowarea = cv2.bitwise_and(img, img, mask = yellowmask) #利用掩膜生成带有颜色的图像
    img=yellowarea
    img = cv2.GaussianBlur(img, (3, 3), 0)
    edges = cv2.Canny(img, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, 300, 5)
    lines_k,lines_j=switch_lines(lines)#k--normal,j--inf
    if len(lines_k)>1:
        lines_k=kmeans1(lines_k,img)
    if lines_j:
        lines_j=kmeans2(lines_j,img)
    lines_k=scale_elements(lines_k,100)
    if lines_j:
        lines_j=scale_elements(lines_j,1000)
        lines=lines_k+lines_j
    else:
        lines=lines_k
    theta=[]
    for k in lines_k:
        angle = math.degrees(math.atan(k[0]))
        theta.append(angle)
    print(theta)
    perpendicular_lines = 0
    err=10
    cnt=0
    for i in range(len(theta)):
        for j in range(i + 1, len(theta)):
            if abs(theta[i] - theta[j]) < 90+err and abs(theta[i] - theta[j]) > 90-err:
                return math.floor(abs(min(theta[i],theta[j])))
                # cnt=cnt+1
			
    return 
                # perpendicular_lines=perpendicular_lines+theta[i]+theta[j]
# image_path = "D:\programme\Robogame\Vis\img\img4.png"
# angle = calculate_angle(cv2.imread(image_path))
# print("正方体相对水平的倾斜角：", angle)
def check_line_sum(yellowarea, x, y, line_length, threshold_ratio):
    # 确保是二值化处理
    binary_yellowarea = (yellowarea > 0).astype(np.uint8)

    # 计算线段的起始和结束坐标
    start_x = x
    end_x = min(x + line_length, yellowarea.shape[1])  # 防止越界

    # 提取线段上的像素值
    line_segment_values = binary_yellowarea[y, start_x:end_x]

    # 计算线段上点的值之和
    sum_values = np.sum(line_segment_values)

    # 判断和是否达到线段长度的0.8
    if sum_values >= (line_length * threshold_ratio):
        print(True)
        return True
    else:
        print(False)
        return False
def check_line_sum_y(yellowarea, x, y, line_length, threshold_ratio):
    # 确保是二值化处理
    binary_yellowarea = (yellowarea > 0).astype(np.uint8)

    # 计算线段的起始和结束坐标
    start_y = y
    end_y = min(y + line_length, yellowarea.shape[1])  # 防止越界

    # 提取线段上的像素值
    line_segment_values = binary_yellowarea[start_y:end_y, x]

    # 计算线段上点的值之和
    sum_values = np.sum(line_segment_values)

    # 判断和是否达到线段长度的0.8
    if sum_values >= (line_length * threshold_ratio):
        print(True)
        return True
    else:
        print(False)
        return False

def calculate_average_coordinates(yellowarea):
	binary_yellowarea = (yellowarea > 0).astype(np.uint8)
	# cv2.imshow("binary_yellowarea", binary_yellowarea)
	# 找到所有值为1的点的坐标,且纵坐标小于720-65=655
	i=0
	y_indices, x_indices,_ = np.where(binary_yellowarea > 0)
	# y_indices, x_indices = [0],[0]
	# print(np.where(binary_yellowarea > 0))
	# for i in range(len(y_indices)):
	# 	if y_indices[i]>655:
	# 		y_indices=np.delete(y_indices,i)
	# y_indices, x_indices ,_= np.where(binary_yellowarea >0)
	if len(y_indices) == 0:  # 如果没有值为1的点
		return None, None
		
	# 计算平均横坐标和纵坐标
	average_x = np.mean(x_indices)
	average_y = np.mean(y_indices)
	print("average_x:", average_x, "average_y:", average_y)	
	return average_x, average_y


def read_angle(env: policy.MyEnv):
    #cap = cv2.VideoCapture("/dev/back_video")
    back_cam = cv2.VideoCapture(7)
    back_cam.set(cv2.CAP_PROP_EXPOSURE,-6)
#self.back_cap = cv2.VideoCapture("/dev/back_video")
    mergin = 25
    dis = 5
    dis2 = 2
    dis_front=1
    cnt=0
    x_low=320
    x_high=370
    y_low=225
    y_high=260
    time_sleep=0.1
    flag=0
    ft_times=3
    max_times=120
    x_max_times=60
    done_time=0

    is_no_target=0
    while True:
        # time.sleep(1)
        print("times:",cnt)
        cnt+=1
        ret, img = back_cam.read()
        img=img[0:437,:,:]
        
        cv2.imshow("img", img)
        cv2.waitKey(1)
        if ret:
            done_time+=1
            if done_time>max_times:
                break
            yellowlow = np.array([0, 50, 100])
            yellowhigh = np.array([35, 255, 255])
            hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            yellowmask = cv2.inRange(hsvimg, yellowlow, yellowhigh)
            yellowarea = cv2.bitwise_and(img, img, mask=yellowmask)
            cv2.imshow("yellowarea", yellowarea)
            cv2.waitKey(1)
            # time.sleep(0.5)
            # 二值化
            ret, binary_yellowarea = cv2.threshold(yellowmask, 127, 255, cv2.THRESH_BINARY)
            # 计算面积
            area = cv2.countNonZero(binary_yellowarea)
            #if area<500,break
            if area<500:
                print("area too small,times:",cnt)
                if is_no_target>ft_times:
                    break
                if cnt==1:
                    command = b'\x66\x66\x08'
                    env.ser.write(command)
                    #print(f"发送数据: {command.hex()}")
                    command = b'\x66\x66\x02'
                    env.ser.write(command)
                    #print(f"发送数据: {command.hex()}")
                    command = b'\x00\x00'
                    env.ser.write(command)
                    #print(f"发送数据: {command.hex()}")
                    env.ser.write(dis.to_bytes(1, byteorder='big'))
                    is_no_target+=1
                    continue
                else:
                    command = b'\x66\x66\x08'
                    env.ser.write(command)
                    #print(f"发送数据: {command.hex()}")
                    command = b'\x66\x66\x02'
                    env.ser.write(command)
                    #print(f"发送数据: {command.hex()}")
                    command = b'\x00\x5A'
                    env.ser.write(command)
                    #print(f"发送数据: {command.hex()}")
                    env.ser.write(dis.to_bytes(1, byteorder='big'))
                    is_no_target+=1
                    continue
            x_done = 0
            y_done = 0
            average_x, average_y = calculate_average_coordinates(yellowarea)

            if average_x is None or average_y is None:
                is_no_target+=1
            else:
            
                
                if x_done == 0 and flag<x_max_times:
                    if average_x < x_low - mergin:  # move 270
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x01\x0E'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        env.ser.write(dis.to_bytes(1, byteorder='big'))
                        print(f"发送数据: {dis.to_bytes(1, byteorder='big').hex()}")


                    elif average_x > x_high + mergin:  # move 90
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x00\x5A'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        env.ser.write(dis.to_bytes(1, byteorder='big'))
                        print(f"发送数据: {dis.to_bytes(1, byteorder='big').hex()}")

                    elif average_x < x_low:
                        env.ser.write(b'\x66\x66\x07')
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x01\x0E'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        env.ser.write(dis2.to_bytes(1, byteorder='big'))
                        print(f"发送数据: {dis2.to_bytes(1, byteorder='big').hex()}")
                        time.sleep(time_sleep)
                        # x_done=1
                    elif average_x > x_high:
                        env.ser.write(b'\x66\x66\x07')
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x00\x5A'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        env.ser.write(dis2.to_bytes(1, byteorder='big'))
                        print(f"发送数据: {dis2.to_bytes(1, byteorder='big').hex()}")
                        time.sleep(time_sleep)
                        # x_done=1
                    else:
                        command = b'\x66\x66\x07'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        x_done = 1
                        flag+=1
                if (y_done == 0 and x_done==1) or flag>0:
                    if average_y < y_low - mergin:  # move 180
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x00\xB4'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        env.ser.write(dis.to_bytes(1, byteorder='big'))
                        print(f"发送数据: {dis.to_bytes(1, byteorder='big').hex()}")
                    elif average_y > y_high + mergin:  # move 0
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x00\x00'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        env.ser.write(dis.to_bytes(1, byteorder='big'))
                        print(f"发送数据: {dis.to_bytes(1, byteorder='big').hex()}")


                    elif average_y < y_low:  # move 180
                        env.ser.write(b'\x66\x66\x07')
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x00\xB4'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        env.ser.write(dis2.to_bytes(1, byteorder='big'))
                        print(f"发送数据: {dis_front.to_bytes(1, byteorder='big').hex()}")
                        time.sleep(time_sleep)
                        # y_done=1
                    elif average_y > y_high:  # move 0
                        env.ser.write(b'\x66\x66\x07')
                        command = b'\x66\x66\x08'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x66\x66\x02'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        command = b'\x00\x00'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        env.ser.write(dis2.to_bytes(1, byteorder='big'))
                        print(f"发送数据: {dis_front.to_bytes(1, byteorder='big').hex()}")
                        time.sleep(time_sleep)
                        # y_done=1
                    else:
                        
                        command = b'\x66\x66\x07'
                        env.ser.write(command)
                        print(f"发送数据: {command.hex()}")
                        y_done = 1
        # with lock_serial:
            if x_done == 1 and y_done == 1:
                command = b'\x66\x66\x07'
                
                env.ser.write(command)
                print(f"发送数据: {command.hex()}")
                # print("fine_tuning done")
                break
        else:
            if cnt%50==0:
                print("无法读取图像")
            ret, img = back_cam.read()
            cnt+=1
            if cnt>100:
                angle=0
                break
        
    command = b'\x66\x66\x05'
    # with lock_serial:
    env.ser.write(command)
if __name__ == '__main__':
    env = policy.MyEnv()
    while True:
        read_angle(env)
        time.sleep(5)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break