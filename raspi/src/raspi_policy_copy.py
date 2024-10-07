import struct
import gymnasium as gym

from gymnasium import spaces

import numpy as np

import time 

import random

import cv2

import math

import heapq

import serial
import threading
import queue
import get_theta
import SGBM_raspi
camera_direction =0

size_env = (300,400)

target_len =15

agent_len =60

num_add_target=10
lock=threading.Lock()
def remove_straight_points(path):

    # 确保路径至少有三个点

    if len(path) < 3:

        return path



    # 初始化结果路径

    simplified_path = [path[0]]



    # 遍历路径中的点

    for i in range(1, len(path) - 1):

        # 计算当前点与前后点的方向向量

        prev_point = path[i - 1]

        curr_point = path[i]

        next_point = path[i + 1]



        # 计算方向向量

        direction_prev = np.array(curr_point) - np.array(prev_point)

        direction_next = np.array(next_point) - np.array(curr_point)



        # 计算方向向量的夹角

        dot_product = np.dot(direction_prev, direction_next)

        norm_product = np.linalg.norm(direction_prev) * np.linalg.norm(direction_next)

        angle = np.arccos(dot_product / norm_product)



        # 如果夹角不为零，说明有转向，保留当前点

        if not np.isclose(angle, 0):

            simplified_path.append(curr_point)



    # 添加最后一个点

    simplified_path.append(path[-1])



    return simplified_path



class MyEnv(gym.Env):

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}


    #TODO get the init location of target


    init_agent_locat=[150,340]
    init_target_locat=([110,32],[151,32],[192,32],[32,131],[31,170],[31,212]
                       ,[31,249],[268,121],[268,161],[268,200])
    def __init__(self, size=5):
        self.serialinit()
        self.size = size  # The size of the square grid
        self.window_size = 400  # The size of the PyGame window
        self.reset()
        

    def find_xmax_ymin_target(self,index):

        max_target_index = -1

        

        turning_dis=43

        while(max_target_index==-1):

            if self.catch_dir==0:

                max_x = -300

                max_y = -400

                max_target = [max_x, max_y]

                flag=-1

                for i, target in enumerate(self._target_location):

                    # if target[0] > max_x or (target[0] == max_x and target[1] > max_y):
                    if self.heuristic(self._agent_location, self._target_location[i]) < self.heuristic(self._agent_location, max_target):

                        # max_target_index = i

                        flag=i

                        max_target = self._target_location[i]

                        left_boundary = max_target[0] +target_len/2

                        right_boundary = max_target[0] + 60+target_len/2

                        top_boundary = max_target[1] + 30

                        bottom_boundary = max_target[1] - 30

                        if right_boundary > 300 or top_boundary > 400 or bottom_boundary < 0:

                            flag=-1

                            continue

                        for target1 in self._target_location:

                            if target1[0] > left_boundary-target_len/2 and target1[0] < right_boundary+target_len/2:

                                if target1[1] < top_boundary+target_len/2 and target1[1] > bottom_boundary-target_len/2:

                                    flag = -1

                                    break

                        for j in index:

                            if j == max_target_index:

                                flag=-1

                                break

                        if flag!=-1:

                            max_target_index = i

                            max_x = target[0]

                            max_y = target[1]

            elif self.catch_dir==3:#*turn around

                max_x = -300

                max_y = -400

                max_target = [max_x, max_y]

                flag=-1

                for i, target in enumerate(self._target_location):

                    if self.heuristic(self._agent_location, self._target_location[i]) < self.heuristic(self._agent_location, max_target):

                        # max_target_index = i

                        flag=i

                        max_target = self._target_location[i]

                        right_boundary = max_target[0] -target_len/2

                        left_boundary = max_target[0] - 30-turning_dis-target_len/2

                        top_boundary = max_target[1] + turning_dis

                        bottom_boundary = max_target[1] - turning_dis

                        if left_boundary < 0 or top_boundary > 400 or bottom_boundary < 0:

                            flag=-1

                            continue

                        for target1 in self._target_location:

                            if target1[0] > left_boundary-target_len/2 and target1[0] < right_boundary+target_len/2:

                                if target1[1] < top_boundary+target_len/2 and target1[1] > bottom_boundary-target_len/2:

                                    flag = -1

                                    break

                        for j in index:

                            if j == max_target_index:

                                flag=-1

                                break

                        if flag!=-1:

                            max_target_index = i

                            max_x = target[0]

                            max_y = target[1]
            elif self.catch_dir==1:#*below the square 

                max_x = -300

                max_y = -400

                max_target = [max_x, max_y]

                flag=-1

                for i, target in enumerate(self._target_location):

                    #nearer to the agent

                    if self.heuristic(self._agent_location, self._target_location[i]) < self.heuristic(self._agent_location, max_target):

                        # max_target_index = i

                        flag=i

                        max_target = self._target_location[i]

                        bottom_boundary = max_target[1] +target_len/2

                        top_boundary = max_target[1] + 30+turning_dis +target_len/2

                        right_boundary = max_target[0] + turning_dis

                        left_boundary = max_target[0] - turning_dis

                        if right_boundary > 300 or top_boundary > 400 or left_boundary < 0:

                            flag=-1

                            continue

                        for target1 in self._target_location:

                            if target1[0] > left_boundary-target_len/2 and target1[0] < right_boundary+target_len/2:

                                if target1[1] < top_boundary+target_len/2 and target1[1] > bottom_boundary-target_len/2:

                                    flag = -1

                                    break

                        for j in index:

                            if j == max_target_index:

                                flag=-1

                                break

                        if flag!=-1:

                            max_target_index = i

                            max_x = target[0]

                            max_y = target[1]

            elif self.catch_dir==2:#*above the square

                max_x = -300

                max_y = -400

                max_target = [max_x, max_y]

                flag=-1

                for i, target in enumerate(self._target_location):

                    if self.heuristic(self._agent_location, self._target_location[i]) < self.heuristic(self._agent_location, max_target):

                        # max_target_index = i

                        flag=i

                        max_target = self._target_location[i]

                        bottom_boundary = max_target[1] -target_len/2-30-turning_dis

                        top_boundary = max_target[1] -target_len/2

                        right_boundary = max_target[0] + turning_dis

                        left_boundary = max_target[0] - turning_dis

                        if right_boundary > 300 or left_boundary < 0 or bottom_boundary < 0:

                            flag=-1

                            continue

                        for target1 in self._target_location:

                            if target1[0] > left_boundary-target_len/2 and target1[0] < right_boundary+target_len/2:

                                if target1[1] < top_boundary+target_len/2 and target1[1] > bottom_boundary-target_len/2:

                                    flag = -1

                                    break

                        for j in index:

                            if j == max_target_index:

                                flag=-1

                                break

                        if flag!=-1:

                            max_target_index = i

                            max_x = target[0]

                            max_y = target[1]

            
            elif self.catch_dir==4:

                return [-1,-1],-1

            if max_target_index == -1:

                self.catch_dir=self.catch_dir+1

        return [max_x,max_y], max_target_index



    def heuristic(self,a, b):

        return abs(a[0] - b[0]) + abs(a[1] - b[1])



    def astar(self,array, start, goal):

        start = tuple(map(int, start))

        goal = tuple(map(int, goal))

        neighbors = [(0,1),(0,-1),(1,0),(-1,0)]

        close_set = set()

        came_from = {}

        gscore = {start:0}

        fscore = {start:self.heuristic(start, goal)}

        oheap = []



        heapq.heappush(oheap, (fscore[start], start))

        

        while oheap:

            current = heapq.heappop(oheap)[1]

            if current == goal:

                data = []

                while current in came_from:

                    data.append(current)

                    current = came_from[current]

                return data[::-1]  # 返回路径时需要反转



            close_set.add(current)

            for i, j in neighbors:

                neighbor = (current[0] + i, current[1] + j)

                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)

                if 0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1] and array[int(neighbor[0])][int(neighbor[1])] == 0:

                    if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):

                        continue

                    

                    if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:

                        came_from[neighbor] = current

                        gscore[neighbor] = tentative_g_score

                        fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                        heapq.heappush(oheap, (fscore[neighbor], neighbor))

                    

        return []

    def make_map(self):

        map = np.zeros((300, 400))

        target_len = self._target_length

        for target in self._target_location:

            for i in range(int(target[0] - target_len / 2-agent_len/2), int(target[0] + target_len / 2+agent_len/2)):

                for j in range(int(target[1] - target_len / 2-agent_len/2), int(target[1] + target_len / 2+agent_len/2)):

                    if i>=0 and i<300 and j>=0 and j<400:

                        map[i][j] = 1

        #render map

        return map





    def A_star_path_planning(self):

        path = []

        index = []

        self.catch_dir=0

        while len(path)==0:

            

            target,i=self.find_xmax_ymin_target(index)

            if i==-1:

                #! dir==4 raise interpution
                
                return [],[]

            #todo: determine target location

            index.append(i)

            map=self.make_map()

            arm_length=5
            with lock:
                if(self.catch_dir==0):
                    goal=[int(target[0]+target_len/2+agent_len/2)+arm_length+1,int(target[1])]
                    path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]+target_len/2+agent_len/2)+arm_length+1,int(target[1])])

                elif(self.catch_dir==1):
                    goal=[int(target[0]),int(target[1]+target_len/2+agent_len/2)+1+arm_length]
                    path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]),int(target[1]+target_len/2+agent_len/2)+1+arm_length])

                elif(self.catch_dir==2):
                    goal=[int(target[0]),int(target[1]-target_len/2-agent_len/2)+arm_length-1]
                    path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]),int(target[1]-target_len/2-agent_len/2)+arm_length-1])

                elif(self.catch_dir==3):
                    goal=[int(target[0]-target_len/2-agent_len/2)+arm_length-1,int(target[1])]
                    path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]-target_len/2-agent_len/2)+arm_length-1,int(target[1])])

            
            if(len(index)==self.num_squares and path==[]):
                self.catch_dir=self.catch_dir+1
                index=[]

        delta=np.subtract(target,self._agent_location)
        if delta[0]==0:
            if delta[1]>0:
                theta=270
            else:
                theta=90
        else:
            theta=math.atan(abs(delta[1])/abs(delta[0]))
            if delta[1]<0 and delta[0]<0:
                theta=theta+90
            elif delta[1]>0 and delta[0]<0:
                theta=theta+180
            elif delta[1]>0 and delta[0]>0:
                theta=theta+270
        self.camera_dir=(math.floor((theta%90)/45)+math.floor(theta/90))*90
        self.change_camera_dir(self.camera_dir)
        #TODO get the pos of target
            #delete target
        self._target_location=np.delete(self._target_location,index,axis=0)
        self.num_squares=self.num_squares-1
        path=remove_straight_points(path)
        print(path)
        return path,goal

    def change_camera_dir(self,angle):
        #TODO commute to stm to change
        self.com_cam_rotate(angle)
        self.camera_dir=angle
    def compare_tar_in_scope(self,target_loca,temp):
        new_target1 = np.array(target_loca)
        for target in temp:
            if np.linalg.norm(new_target1-target)<10:
                self.add_certain_target((new_target1+target)/2)
                return True
        self.add_certain_target(new_target1)
        return False


        #a=fun(self) print(a["agent"])
    def _get_obs(self):
        #! not remove all targets ,only to compare
        with lock:
            temp_target_location=np.array([])
            for target in self._target_location:

                delta=target-self._agent_location

                if delta[0]==0:

                    if delta[1]>0:

                        theta=90

                    else:

                        theta=270

                    continue

                else:

                    theta=math.atan(abs(delta[1])/abs(delta[0]))

                    if delta[1]<0 and delta[0]<0:

                        theta=theta+270

                    elif delta[1]>0 and delta[0]>0:

                        theta=theta+90

                    elif delta[1]<0 and delta[0]>0:

                        theta=theta+180

                theta_scope=math.atan(65/80)

                if theta<(self.camera_dir+theta_scope)%360 and theta> (360+(self.camera_dir-theta_scope))%360:

                    index = np.where((self._target_location == target).all(axis=1))
                    temp_target_location=np.append(temp_target_location,target,axis=0)
                    self._target_location = np.delete(self._target_location, index, axis=0)
                    self.num_squares=self.num_squares-1
        #!camera dis needs to move 3cm to the shooting edge
        pos_err_x=3
        pos_err_y=0
        #! SGBM
        targets=SGBM_raspi.find_target_loc()
        for target in targets:
            x=target[0][0]
            y=target[0][1]
            with lock:
                if self.camera_dir==0:
                    target_loc=[self._agent_location[0]-x-pos_err_x,self._agent_location[1]-y]
                elif self.camera_dir==90:
                    target_loc=[self._agent_location[0]-y-pos_err_x,self._agent_location[1]+x]
                elif self.camera_dir==180:
                    target_loc=[self._agent_location[0]+x-pos_err_x,self._agent_location[1]+y]
                elif self.camera_dir==270:
                    target_loc=[self._agent_location[0]+y-pos_err_x,self._agent_location[1]-x]
                if not (target_loc[0]<20 or target_loc[0]>280 or target_loc[1]<20 or target_loc[1]>380):
                    self.compare_tar_in_scope(target_loc,temp_target_location)
            
    def reset(self, seed=None, return_info=False, options=None):
        super().reset(seed=seed)
        self._agent_length=agent_len
        self._agent_location = np.array(self.init_agent_locat)
        self.init_target()
        self.camera_dir=0
        self.catch_dir=0
    def init_target(self):
        self._target_length=target_len
        self._target_location = np.array(self.init_target_locat)
        self.num_squares=len(self.init_target_locat)
        return(self._target_length,self.num_squares,self._target_location)
    def add_certain_target(self,target_loca):
        self.num_squares=self.num_squares+1
        self._target_location = np.append(self._target_location, target_loca, axis=0)
        return(self._target_length,self.num_squares,self._target_location)
    # def add_ran_target(self,num_squares):
    #     self._target_length=target_len
    #     new_target = np.random.randint(low=0, high=300-15, size=(num_squares, 2))
    #     # 为了确保y坐标小于400，我们需要单独处理y坐标
    #     new_target[:, 1] = np.random.randint(low=0, high=400-15, size=10)
    #     self._target_location = np.append(self._target_location, new_target, axis=0)
    #     self._target_angles = np.random.randint(0, 90, size=num_squares)
    #     self.num_squares=self.num_squares+num_squares
        # return(self._target_length,self.num_squares,self._target_location,self._target_angles)
    def serialinit(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
        except:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.5)
        self.ser.bytesize = 8
        print("serial_init_success")
        return self.ser
    def com_cam_rotate(self,angle):

            pass

        #self.ser.write(b'\x66\x66\x01')

        #TODO angle is verse clockwise

    def com_move(self):
        path,goal=self.A_star_path_planning()
        if path!=[]:
            for i in range(len(path)-1):
                self.ser.write(b'\x66\x66\x02')
                if path[i][1]-path[i+1][1] > 0:
                    dis=path[i][1]-path[i+1][1]
                    self.ser.write(b'\x01\x0E')
                    self.ser.write(dis.to_bytes(1, byteorder='big'))
                elif path[i][1]-path[i+1][1] < 0:
                    dis=path[i+1][1]-path[i][1]
                    self.ser.write(b'\x00\x5A')
                    self.ser.write(dis.to_bytes(1, byteorder='big'))
                elif path[i][0]-path[i+1][0] < 0:
                    dis=path[i+1][0]-path[i][0]
                    self.ser.write(b'\x00\xB4')
                    self.ser.write(dis.to_bytes(1, byteorder='big'))
                elif path[i][0]-path[i+1][0] > 0:
                    dis=path[i][0]-path[i+1][0]
                    self.ser.write(b'\x00\x00')
                    self.ser.write(dis.to_bytes(1, byteorder='big'))
            if self.catch_dir==1:
                self.com_car_rotate(90)
                self.ser.write(b'\x66\x66\x05')
                self.com_car_rotate(270)
            elif self.catch_dir==2:
                self.com_car_rotate(270)
                self.ser.write(b'\x66\x66\x05')
                self.com_car_rotate(90)
            elif self.catch_dir==3: 
                self.com_car_rotate(180)
                self.ser.write(b'\x66\x66\x05')
                self.com_car_rotate(180)
            return goal 
        else:
            self.ser.write(b'\x66\x66\x02')
            self.ser.write(b'\x00\xB4\x32')
            
    def com_car_rotate(self,angle):
        self.ser.write(b'\x66\x66\x03')
        #*angle is verse clockwise
        self.ser.write(angle.to_bytes(2, byteorder='big'))
    #TODO: 增加对stm的控制射速
    def recieve_data(self):
        while True:
    # 读取3个字节的数据
            data = self.ser.readline()
            if data:
                print("success receive data:",data)
                # 打印接收到的数据
                if data[0] == 0x66 and data[1] == 0x66 :
                    if data[2] == 0x82:
                        angle=False
                        while not angle:
                                angle=get_theta.read_angle ()
                        self.ser.write(b'\x66\x66\x01')
                        self.ser.write(angle.to_bytes(2, byteorder='big'))
                        self.ser.write(b'\x66\x66\x06')
                        #goal=self.com_move()

                        

# def run_policy(env:MyEnv,q,q2):
#     while True:
#         if not q.empty():
#             new_target = q.get()
#             if new_target is None:
#                 env.com_move
#                 print("no new target")
#                 time.sleep(2)
#                 break
#         # 处理新目标的逻辑
#             # env._target_location=np.append(env._target_location, new_target, axis=0)
#             # env.num_squares=env.num_squares+1
#             print("Processing target:", new_target)
#             time.sleep(0.5)
#         if not q2.empty():
#             char=q2.get()
#             print("get:",char)

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

def auto_add(env:MyEnv,q,q2,q3):
    goal=[0,0]
    while True:
        if not q.empty():
            data=q.get()
            if data is None:
                if data[0] == 0x66 and data[1] == 0x66 :
                    if data[2] == 0x82:
                        env._agent_location=np.array(goal)
                        q3.put(data)
                        goal=env.com_move()
                        #!q2.put(agent_loc)
                        env._get_obs()
                        env.change_camera_dir(env.camera_dir+90)
def back_camera(env:MyEnv,q3):
    cap=cv2.VideoCapture("/dev/back_video")
    while True:
        if not q3.empty():
            data=q3.get()
            print("get_data")
            if data is None :
                ret,frame=cap.read()
                if ret:
                    img=frame
                    print("get_back_img")
                    yellowlow = np.array([10, 60, 80])
                    yellowhigh = np.array([100, 255, 255])
                    hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #将BGR色彩空间转化为HSV色彩空间 
                    yellowmask = cv2.inRange(hsvimg, yellowlow, yellowhigh) #提取掩膜 
                    yellowarea = cv2.bitwise_and(img, img, mask = yellowmask) #利用掩膜生成带有颜色的图像
                    img=yellowarea
                    # contours, hirrarchy = cv2.findContours(yellowmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    # cv2.imshow('image1', hsvimg)
                    dis=1
                    while True:

                        if not check_line_sum_y(yellowarea, 630,120,240,0.8):
                                        #move to 270 degree
                            env.ser.write(b'\x01\x0E')
                            env.ser.write(dis.to_bytes(1, byteorder='big'))
                        elif not check_line_sum_y(yellowarea, 150,120,240,0.8):
                            env.ser.write(b'\x00\x5A')
                            env.ser.write(dis.to_bytes(1, byteorder='big'))
                            
                        else:
                            if check_line_sum(yellowarea, 170,33,300,0.8):
                                    #move to 180 degree
                                    env.ser.write(b'\x00\xB4')
                                    env.ser.write(dis.to_bytes(1, byteorder='big'))
                            elif not check_line_sum(yellowarea, 170,156,300,0.8):
                                    # move to 0 degree
                                    env.ser.write(b'\x00\x00')
                                    env.ser.write(dis.to_bytes(1, byteorder='big'))
                            else:
                                break

                else:
                    print("无法读取图像")
                    break
                #!wait for next catch
                # 显示图像

                # 按'q'键退出循环
                angle=get_theta.calculate_angle(img)
                env.ser.write(b'\x66\x66\x01')
                env.ser.write(angle.to_bytes(2, byteorder='big'))
                env.ser.write(b'\x66\x66\x06')
def comera_compute(q2):
    left_camera_matrix = np.array([[6.634196771656173e+02,0,6.453381335439782e+02],
                                [0,6.627287404108016e+02,3.317645687022463e+02],
                                [0,0,1]])
    # left_camera_matrix = np.array([[516.5066236,-1.444673028,320.2950423],[0,516.5816117,270.7881873],[0.,0.,1.]])
    right_camera_matrix = np.array([[6.646011676487399e+02,0,6.427265982999328e+02],
                                    [0,6.641576948267607e+02,3.529921377190129e+02],
                                    [0,0,1]])

    # 畸变系数,K1、K2、K3为径向畸变,P1、P2为切向畸变
    # left_distortion = np.array([[-0.081929747426762,0.318162062028180, -4.621039944244846e-04,0.001040637933566,0]])
    # right_distortion = np.array([[-0.083580953942706,0.250195130695020,3.070102124682274e-04,9.696705014779844e-04,0]])
    left_distortion = np.array([[-0.079351676446754,0.328815720032369,-3.590462881309750e-04,3.288008180493479e-04,-0.823761273632520]])
    right_distortion = np.array([[-0.082168598853966,0.269750168613135,2.770800154867922e-04,-0.001049916133447,0.124881117739656]])

    # 旋转矩阵
    # R = np.array([[0.999911333,-0.004351508,0.012585312],
    #               [0.004184066,0.999902792,0.013300386],
    #               [-0.012641965,-0.013246549,0.999832341]])
    R = np.array([[0.999421555984420,-0.027382812481010,0.020167672506016],
                [0.028055822910123,0.999032028381329,-0.033880334548577],
                [-0.019220411923678,0.034426557320150,0.999222391620888]])
    # 平移矩阵
    # T = np.array([-120.3559901,-0.188953775,-0.662073075])
    T = np.array([1.499374167536796e+02,3.676463434972824,-2.875770681519952])
    cap = cv2.VideoCapture("/dev/r_video") #r
    cap2 = cv2.VideoCapture("/dev/l_video")#l
# fps = 0.0
    width=1280
    height=720
    cap.set(3, width)  # 设置宽度为640
    cap.set(4, height)  # 设置高度为480
    cap2.set(3, width)  # 设置宽度为640
    cap2.set(4, height)  # 设置高度为480
    # cap.set(cv2.CAP_PROP_EXPOSURE, 20)  #设置曝光值 1.0 - 5000  156.0
    # cap2.set(cv2.CAP_PROP_EXPOSURE, 20)  #设置曝光值 1.0 - 5000  156.0
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
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
    while True:
        if not q2.empty():
            agent_location=q2.get()
            if agent_location is None:
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
                    boxes=SGBM_raspi.find_yellow_boxes(frame2,dis_color)
                    return SGBM_raspi.cal_average_distance_in_boxes(boxes,threeD)
                    
                else:
                    print("No frame!")
                    cap.release()
                    cap2.release()
                    cap = cv2.VideoCapture("/dev/r_video") #r
                    cap2 = cv2.VideoCapture("/dev/l_video")#l


def com_to_STM(env:MyEnv,q):
    while True:
    # 读取3个字节的数据
        data = env.ser.readline()
        if data:
            print("success receive data:",data)
            q.put(data)


#         with open('D:/programme/Robogame/policy/transmit/1.txt', 'r') as file:
# # 读取文件中的所有内容
#             content = file.read()
#             char=content.split()[0]
#             q.put(char)
#             print("input:",char)



if __name__=="__main__":
    env=MyEnv()
    env.reset()
    env.com_move()
    # env.add_ran_target(10)
    # env.render()
    q = queue.Queue()
    q2= queue.Queue()
    q3= queue.Queue()
    t1 = threading.Thread(target=auto_add, args=(env,q,q2,q3))
    t3 = threading.Thread(target=com_to_STM, args=(env,q))
    t2 = threading.Thread(target=back_camera, args=(env,q3))

    t2.start()
    t1.start()
    t3.start()
    t1.join()
    t3.join()
    env.close()
