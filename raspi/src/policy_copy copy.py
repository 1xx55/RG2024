# import struct
# import gymnasium as gym
# from gymnasium import spaces
import numpy as np
import time 
# import random
import cv2
import math
import heapq
import SGBM_raspi
import get_theta
import serial
import threading
import queue
import apriltag.Apriltag_windows as Apriltag
# import get_theta
lock=threading.Lock()
lock_serial=threading.Lock()
lock_queue=threading.Lock()

camera_direction =0
size_env = (300,400)
target_len =12
agent_len =56
# num_add_target=10
# signcv=cv2.CAP_DSHOW
# front_cam_index=1+signcv

# back_left_cam_index=3+signcv
# back_right_cam_index=0+signcv
# back_cam_index=7+signcv

# battery_left_cam_index=8+signcv
# battery_right_cam_index=5+signcv

# pump_left_cam_index=2+signcv
# pump_right_cam_index=4+signcv

front_cam_index=1

back_left_cam_index=3
back_right_cam_index=0
back_cam_index=7

battery_left_cam_index=8
battery_right_cam_index=4

pump_left_cam_index=2
pump_right_cam_index=5

serial_index='com7'

pump_light=-4
battery_left_light=-4
battery_right_light=-5
back_left_light=-6
back_right_light=-5
back_FT_light=-6
front_light=-5
print("init_cam")
# back_left_cam=cv2.VideoCapture(back_left_cam_index)
# back_right_cam=cv2.VideoCapture(back_right_cam_index)
# battery_left_cam=cv2.VideoCapture(battery_left_cam_index)
# battery_right_cam=cv2.VideoCapture(battery_right_cam_index)
# pump_left_cam=cv2.VideoCapture(pump_left_cam_index)
# pump_right_cam=cv2.VideoCapture(pump_right_cam_index)
# width=1280
# height=720
# back_left_cam.set(3, width)  # 设置宽度为640
# back_left_cam.set(4, height)  # 设置高度为480
# back_right_cam.set(3, width)  # 设置宽度为640
# back_right_cam.set(4, height)  # 设置高度为480    
# back_right_cam.set(cv2.CAP_PROP_EXPOSURE,back_right_light)  # back_right -6
# back_left_cam.set(cv2.CAP_PROP_EXPOSURE,back_left_light)  # back_left -5
# re_open_cnt=0     
# if back_left_cam.isOpened() and back_right_cam.isOpened():

#     ret, frame1 = back_right_cam.read()
#     ret2, frame2 = back_left_cam.read()
#     if  ret and ret2 :
#         imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
#         imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
#         print("back_cam init success!")
# width=1280
# height=720
# battery_left_cam.set(3, width)  # 设置宽度为640
# battery_left_cam.set(4, height)  # 设置高度为480
# battery_right_cam.set(3, width)  # 设置宽度为640
# battery_right_cam.set(4, height)  # 设置高度为480    
# battery_right_cam.set(cv2.CAP_PROP_EXPOSURE, battery_light)  #设置曝光值 1.0 - 5000  156.0
# battery_left_cam.set(cv2.CAP_PROP_EXPOSURE,battery_light)  #设置曝光值 1.0 - 5000  156.0
# re_open_cnt=0    
# if battery_left_cam.isOpened() and battery_right_cam.isOpened():
#     ret, frame1 = battery_right_cam.read()
#     ret2, frame2 = battery_left_cam.read()

#     if  ret and ret2 :
#         imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
#         imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
#         print("battery_cam init success!")
# width=1280
# height=720
# pump_left_cam.set(3, width)  # 设置宽度为640
# pump_left_cam.set(4, height)  # 设置高度为480
# pump_right_cam.set(3, width)  # 设置宽度为640
# pump_right_cam.set(4, height)  # 设置高度为480  
# pump_right_cam.set(cv2.CAP_PROP_EXPOSURE, pump_light)  #设置曝光值 1.0 - 5000  156.0
# pump_left_cam.set(cv2.CAP_PROP_EXPOSURE,pump_light)  #设置曝光值 1.0 - 5000  156.0  
# re_open_cnt=0     
# if pump_left_cam.isOpened() and pump_right_cam.isOpened() :

#     ret, frame1 = pump_right_cam.read()
#     ret2, frame2 = pump_left_cam.read()
#     if  ret and ret2 :
#         imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
#         imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
#         print("pump_cam init success!")
# front_cam=cv2.VideoCapture(front_cam_index)
# turning_angle=1
# ret,img=front_cam.read()
# if ret:
#     img2=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#     print("front_cam init")
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

class MyEnv():
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}
    init_agent_locat=[147,338]
    #TODO get the init location of target
    # init_target_locat=([110,87],[151,87],[192,87],[87,131],[87,170],[87,210]
    #                     ,[87,240])
                        # ,[220,121],[220,161],[220,200])
    init_target_locat=[[240,40]]
    def __init__(self, size=5):
        self.serialinit()
        self.target_done=[]
        self.size = size  # The size of the square grid
        self.window_size = 400  # The size of the PyGame window
        self.reset()
        #self.cam_init()
        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        
        #! spaces.Box(0, [width,length], shape=(2,), dtype=float)
        self._agent_length=agent_len
        # self._agent_location = self.np_random.integers(0, self.window_size-self._agent_length
        #                                                , size=2, dtype=int)

        #self.ser.write(b'\x66\x66\x04\x04')

        self.catch_dir=0

    def find_xmax_ymin_target(self,index):
        max_target_index = -1
        
        turning_dis=43
        print(self._target_location)
        while(max_target_index==-1):
            if self.catch_dir==0:
                max_x = -300
                max_y = -400
                max_target = [max_x, max_y]
                flag=-1
                for i, target in enumerate(self._target_location):
                    # if target[0] > max_x or (target[0] == max_x and target[1] > max_y):
                         # max_target_index = i
                    if self.heuristic(self._agent_location, self._target_location[i]) < self.heuristic(self._agent_location, max_target):
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
            if self.catch_dir==3:#*turn around
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
            if self.catch_dir==2:#*above the square
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
            if self.catch_dir==1:#*below the square 
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
            if self.catch_dir==4:
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
        agent_locate=self._agent_location
        for target in self._target_location:
            if agent_locate[0]-target_len/2-agent_len/2<target[0] and target[0]<agent_locate[0]+target_len/2+agent_len/2 and agent_locate[1]-target_len/2-agent_len/2<target[1]<agent_locate[1]+target_len/2+agent_len/2:
                self.target_location=np.delete(self.target_location,np.where(self.target_location==target),axis=0)
            else:
                for i in range(int(target[0] - target_len / 2-agent_len/2), int(target[0] + target_len / 2+agent_len/2)):
                    for j in range(int(target[1] - target_len / 2-agent_len/2), int(target[1] + target_len / 2+agent_len/2)):
                        if i>=0 and i<300 and j>=0 and j<400:
                            map[i][j] = 1
        #render map
        return map


    def A_star_path_planning(self):
        # print("start_A_star_path_planning")
        path = []
        index = []
        self.catch_dir=0
        while len(path)==0:
            # print("start find_nearest_target")
            target,i=self.find_xmax_ymin_target(index)
            if i==-1:
                #! dir==4 raise interpution
                print("no path")
                return []
            #todo: determine target location
            index.append(i)
            map=self.make_map()
            
            arm_length=36.5
            if(self.catch_dir==0):
                path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]+target_len/2+arm_length),int(target[1])])
            if(self.catch_dir==1):
                path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]),int(target[1]+target_len/2+arm_length)])
            if(self.catch_dir==2):
                path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]),int(target[1]-target_len/2-arm_length)])
            if(self.catch_dir==3):
                path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]-target_len/2-arm_length),int(target[1])])
            
            
            if(len(index)==self.num_squares and path==[]):
                self.catch_dir=self.catch_dir+1
                index=[]
        # delta=np.subtract(target,self._agent_location)
        # if delta[0]==0:
        #     if delta[1]>0:
        #         theta=270
        #     else:
        #         theta=90
        # else:
        #     theta=math.atan(abs(delta[1])/abs(delta[0]))
        #     if delta[1]<0 and delta[0]<0:
        #         theta=theta+90
        #     elif delta[1]>0 and delta[0]<0:
        #         theta=theta+180
        #     elif delta[1]>0 and delta[0]>0:
        #         theta=theta+270
        # self.camera_dir=(math.floor((theta%90)/45)+math.floor(theta/90))*90
        # self.change_camera_dir(self.camera_dir)
        #     #delete target
            
        self._target_location=np.delete(self._target_location,index,axis=0)
        self.num_squares=self.num_squares-1
        if self.catch_dir==0:
            self._agent_location=[int(target[0]+target_len/2+agent_len/2)+1,int(target[1])]
        if(self.catch_dir==1):
            self._agent_location=[int(target[0]),int(target[1]+target_len/2+agent_len/2)+1]
        if(self.catch_dir==2):
            self._agent_location=[int(target[0]),int(target[1]-target_len/2-agent_len/2)-1]
        if(self.catch_dir==3):
            self._agent_location=[int(target[0]-target_len/2-agent_len/2)-1,int(target[1])]
        path=remove_straight_points(path)
        print("catch_dir_path_Planning:",self.catch_dir)
        # print(path)
        return path

    # def change_camera_dir(self,angle):
    #     #TODO commute to stm to change
    #     delta=self.camera_dir-angle
    #     if delta==180 or delta==-180:
    #         self.ser.write(b'\x66\x66\x04')
    #         self.ser.write(b'\x03')
    #     elif delta==90 or delta==-270:
    #         self.ser.write(b'\x66\x66\x04')
    #         self.ser.write(b'\x02')
    #     elif delta==270 or delta==-90:
    #         self.ser.write(b'\x66\x66\x04')
    #         self.ser.write(b'\x01')
    #     self.camera_dir=angle


        #a=fun(self) print(a["agent"])
    def compare_tar_in_scope(self,target_loca,temp):
        new_target1 = np.array(target_loca)
        temp = np.array(temp)
        for target in temp:
            if np.linalg.norm(new_target1-target)<10:
                #self.add_certain_target((new_target1+target)/2)
                return True
        self.add_certain_target([new_target1])
        return False

    # def get_obs(self,targets,agent_locate,camera_dir):
    #     #! not remove all targets ,only to compare
    #     print("start_get_obs")
    #     with lock:
    #         #if exist self.target_location that is in the scope of the camera, then add it to temp_target_location and delete it from self.target_location
    #         self.camera_dir=camera_dir
    #         temp_target_location=np.array([])
    #         for target in self._target_location:
    #             delta=[0,0]
    #     #a=fun(self) print(a["agent"])
    def _get_obs(self,targets,agent_locate,camera_dir):
        #! not remove all targets ,only to compare
        print("start_get_obs")
        print("agent_locate:",agent_locate)
        print("camera_dir:",camera_dir)
        print("targets:",targets)
        with lock:
            self.camera_dir=camera_dir
            temp_target_location=[]
            for target in self._target_location:
                delta=[0,0]
                delta[0]=int(target[0]-agent_locate[0]) 
                delta[1]=int(target[1]-agent_locate[1])

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

                # if theta<(self.camera_dir+theta_scope)%360 and theta> (360+(self.camera_dir-theta_scope))%360 and not (abs(target[0]-self._agent_location[0])<60 or abs(target[1]-self._agent_location[1])<60):
                if theta<(self.camera_dir+theta_scope)%360 and theta> (360+(self.camera_dir-theta_scope))%360:
                    index = np.where((self._target_location == target).all(axis=1))
                    temp_target_location.append(target)
                    self._target_location = np.delete(self._target_location, index, axis=0)
                    self.num_squares=self.num_squares-1
        #!camera dis needs to move 3cm to the shooting edge
            pos_err_x=8
            pos_err_y=4
            #! SGBM
            # targets=SGBM_raspi.find_target_loc()
            # print(targets)
            for target in targets:
                x=int(target[1]/10)
                y=int(target[0]/10)
                if self.camera_dir==0:
                    target_loc=[agent_locate[0]-x-pos_err_x,agent_locate[1]-y]
                elif self.camera_dir==90:
                    target_loc=[agent_locate[0]-y-pos_err_y,agent_locate[1]+x+20]
                elif self.camera_dir==180:
                    target_loc=[agent_locate[0]+x-pos_err_x,agent_locate[1]+y]
                elif self.camera_dir==270:
                    target_loc=[agent_locate[0]+y-pos_err_x,agent_locate[1]-x-20]
                if not (target_loc[0]<20 or target_loc[0]>280 or target_loc[1]<20 or target_loc[1]>400):
                    self.compare_tar_in_scope(target_loc,temp_target_location)
        print("end_get_obs")
    def reset(self):
        
        self._agent_length=agent_len
        self._agent_location = np.array(self.init_agent_locat)
        self.init_target()
        self.camera_dir=0
        self.catch_dir=0

    def init_target(self):
        self._target_length=target_len
        # self._target_location = np.random.randint(0, self.window_size-self._target_length
        #                                            , size=(num_squares, 2))
        self._target_location = np.array(self.init_target_locat)
        self.num_squares=len(self.init_target_locat)
        # 为了确保y坐标小于400，我们需要单独处理y坐标
        return(self._target_length,self.num_squares,self._target_location)
    def add_certain_target(self,target_loca):

        self.num_squares=self.num_squares+len(target_loca)
# 将新目标添加到目标位置数组中
        self._target_location = np.append(self._target_location, target_loca, axis=0)
        # self._target_location = np.random.randint(0, self.window_size-self._target_length
        #                                            , size=(num_squares, 2))
        return(self._target_length,self.num_squares,self._target_location)





            #!change the format of ser
    def serialinit(self):
        try:
            self.ser = serial.Serial(serial_index, 115200, timeout=0.5)
        except:
            self.ser = serial.Serial('COM10', 115200, timeout=0.5)
        self.ser.bytesize = 8
        print("serial_init_success")
        return self.ser

    def com_move(self):
        print("_________com_move_________")
        # print("self.target_location:",self._target_location)
        print("self.agent_location:",self._agent_location)
        with lock:
            path=self.A_star_path_planning()
        print("path:",path)
        if path==[]:
            return False
        for i in range(len(path)-1):
            self.ser.write(b'\x66\x66\x02')
            if path[i][1]-path[i+1][1] > 0:
                dis=path[i][1]-path[i+1][1]
                self.ser.write(b'\x01\x0E')
                print(dis)
                self.ser.write(dis.to_bytes(1, byteorder='big'))
                print("l")
                print(dis.to_bytes(1, byteorder='big'))
            elif path[i][1]-path[i+1][1] < 0:
                dis=path[i+1][1]-path[i][1]
                self.ser.write(b'\x00\x5A')
                self.ser.write(dis.to_bytes(1, byteorder='big'))
                print(dis)
                print("r")
            elif path[i][0]-path[i+1][0] < 0:
                dis=path[i+1][0]-path[i][0]
                self.ser.write(b'\x00\xB4')
                self.ser.write(dis.to_bytes(1, byteorder='big'))
                print(dis)
                print("f")
            elif path[i][0]-path[i+1][0] > 0:
                dis=path[i][0]-path[i+1][0]
                self.ser.write(b'\x00\x00')
                self.ser.write(dis.to_bytes(1, byteorder='big'))
                print(dis)
                print("b")
        return True
        # if self.catch_dir==1:
        #     self.com_car_rotate(90)
        #     self.ser.write(b'\x66\x66\x05')
        #     self.com_car_rotate(270)
        # elif self.catch_dir==2:
        #     self.com_car_rotate(270)
        #     self.ser.write(b'\x66\x66\x05')
        #     self.com_car_rotate(90)
        # elif self.catch_dir==3: 
        #     self.com_car_rotate(180)
        #     self.ser.write(b'\x66\x66\x05')
        #     self.com_car_rotate(180)
        # else:self.ser.write(b'\x66\x66\x05')

        #ret,frame=self.back_cap.read()
        #if ret:
        #    print("back_video")
        #else:
          #  print("back_video_fail")

    def com_car_rotate(self,angle):
        angle=int(angle)+180
        self.ser.write(b'\x66\x66\x03')
        #*angle is verse clockwise
        self.ser.write(angle.to_bytes(2, byteorder='big'))
    #TODO: 增加对stm的控制射速
    def recieve_data_myversion(self):
        while True:
    # 读取3个字节的数据
            data = self.ser.readline()
            if data:
                # 打印接收到的数据
                print(data)
                # 检查前三个字节是否为0x66
                # if data[0] == 0x66 and data[1] == 0x66 :
                    # 发送两个字节的数据0x66
                    # if data[2] == 0x33:
                    #     env.com_move()
                if data[0] == 0x66 and data[1] == 0x66 :
                    # 发送两个字节的数据0x66
                    if data[2] == 0x82:
                        
                        # angle=False
                        # while not angle:
                        angle=get_theta.read_angle(self)
                        if angle:
                            self.ser.write(b'\x66\x66\x01')
                            self.ser.write(angle.to_bytes(2, byteorder='big'))
                        time.sleep(0.5)
                        self.ser.write(b'\x66\x66\x06')
                        
                        self.com_move()
                        pass
                    if data[2] == 0x33:
                            env.com_move()
def fine_tuning(env:MyEnv,q_FT,q_locate):
    back_cam=cv2.VideoCapture(back_cam_index)
    back_cam.set(cv2.CAP_PROP_EXPOSURE,back_FT_light)
    ret,img=back_cam.read()
    flag82=0
    flag85=1
    if ret:
        img2=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        print("back_success")
    while True:
        # break
        data=q_FT.get()
        if data and data[0] == 0x66 and data[1] == 0x66 and data[2] == 0x82:
            print("get FT_82")
            flag82=1
        if data and data[0] == 0x66 and data[1] == 0x66 and data[2] == 0x85:
            print("get FT_85")
            flag85=1
        if flag82 and flag85:
            flag82=0
            flag85=0
            q_locate.put(b'\x66\x66\x99')
            print("start fine_tuning")
            mergin=25
            dis = 4
            dis2 = 2
            dis_front=1
            cnt=0
            x_low=320
            x_high=370
            y_low=225
            y_high=260
            time_sleep=0
            # time.sleep(0)
            flag=0
            is_no_target=0
            cnt=0
            max_times=70
            max_x_times=40
            done_time=0
            ft_times=4
            gap_time=0.7
            with lock_serial:
                command = b'\x66\x66\x09'
                print("catch_dir_FT:",env.catch_dir)
                env.ser.write(command)
                if env.catch_dir==1:
                    env.com_car_rotate(-90)
                    print("rotate -90")
                    # env.ser.write(b'\x66\x66\x05')
                    # env.com_car_rotate(270)
                elif env.catch_dir==2:
                    env.com_car_rotate(90)
                    print("rotate 90")
                    # env.ser.write(b'\x66\x66\x05')
                    # env.com_car_rotate(90)
                elif env.catch_dir==3: 
                    env.com_car_rotate(180)
                    print("rotate 180")
                    # env.ser.write(b'\x66\x66\x05')
                    # env.com_car_rotate(180)
                t=0
                t1=0
                while True:
                    print("times:",cnt)
                    cnt+=1
                    ret, img = back_cam.read()
                    img=img[0:437,:,:]
                    if ret:
                        done_time+=1
                        if done_time>max_times:
                            break
                        yellowlow = np.array([0, 50, 100])
                        yellowhigh = np.array([35, 255, 255])
                        hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                        yellowmask = cv2.inRange(hsvimg, yellowlow, yellowhigh)
                        yellowarea = cv2.bitwise_and(img, img, mask=yellowmask)
                        
                        # 二值化
                        ret, binary_yellowarea = cv2.threshold(yellowmask, 127, 255, cv2.THRESH_BINARY)
                        # 计算面积
                        area = cv2.countNonZero(binary_yellowarea)
                        #if area<500,break
                        if area<500:
                            print("area too small,times:",cnt)
                            if is_no_target>ft_times:
                                break
                            if cnt%2==0:
                                time.time()
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
                        average_x, average_y = get_theta.calculate_average_coordinates(yellowarea)
                        
                        if average_x is None or average_y is None:
                            is_no_target+=1
                        else:
                        
                            
                            if x_done == 0 and flag<max_x_times:
                                if average_x < x_low - mergin:  # move 270
                                    if t==0 or time.time()-t>gap_time:
                                        t=time.time()
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
                                    if t==0 or time.time()-t>gap_time:
                                        t=time.time()
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
                                    if t==0 or time.time()-t>gap_time:
                                        t=time.time()
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
                                    if t==0 or time.time()-t>gap_time:
                                        t=time.time()
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
                                    if t1==0 or time.time()-t1>gap_time:
                                        t1=time.time()
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
                                    if t1==0 or time.time()-t1>gap_time:
                                        t1=time.time()
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
                                    if t1==0 or time.time()-t1>gap_time:
                                        t1=time.time()
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
                                        env.ser.write(dis_front.to_bytes(1, byteorder='big'))
                                        print(f"发送数据: {dis_front.to_bytes(1, byteorder='big').hex()}")
                                        time.sleep(time_sleep)
                                    # y_done=1
                                elif average_y > y_high:  # move 0
                                    if t1==0 or time.time()-t1>gap_time:
                                        t1=time.time()
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
                                        env.ser.write(dis_front.to_bytes(1, byteorder='big'))
                                        print(f"发送数据: {dis_front.to_bytes(1, byteorder='big').hex()}")
                                        time.sleep(time_sleep)
                                    # y_done=1
                                elif x_done==1:
                                    
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
            # print("end ft_lock")
            with lock_serial:
                
                if is_no_target>ft_times:
                    command = b'\x66\x66\x07'
                    env.ser.write(command)
                    if env.catch_dir==1:
                        # env.com_car_rotate(90)
                        # env.ser.write(b'\x66\x66\x05')
                        print("rotateB 90")
                        env.com_car_rotate(90)
                    elif env.catch_dir==2:
                        # env.com_car_rotate(270)
                        # env.ser.write(b'\x66\x66\x05')
                        print("rotateB -90")
                        env.com_car_rotate(-90)
                    elif env.catch_dir==3: 
                        # env.com_car_rotate(180)
                        # env.ser.write(b'\x66\x66\x05')
                        print("rotateB 180")
                        env.com_car_rotate(180)
                    env.catch_dir=0
                    env.ser.write(command)
                    command = b'\x66\x66\x0A'
                    env.ser.write(command)
                    # with lock_queue:
                    q_locate.put(b'\x66\x66\x98')
                    print("ft_no target")
                    flag85=1
                    #pass
                else:    
                    print("fine_tuning done")
                    command = b'\x66\x66\x05'
                    # with lock_serial:
                    env.ser.write(command)
                    if env.catch_dir==1:
                        # env.com_car_rotate(90)
                        # env.ser.write(b'\x66\x66\x05')
                        print("rotateB 90")
                        env.com_car_rotate(90)
                    elif env.catch_dir==2:
                        # env.com_car_rotate(270)
                        # env.ser.write(b'\x66\x66\x05')
                        print("rotateB -90")
                        env.com_car_rotate(-90)
                    elif env.catch_dir==3: 
                        # env.com_car_rotate(180)
                        # env.ser.write(b'\x66\x66\x05')
                        print("rotateB 180")
                        env.com_car_rotate(180)
                    env.catch_dir=0
                    angle=90
                    # time.sleep(0.8)
                    # ret,img=back_cam.read()
                    # angle=None
                    # if ret:
                    #     angle = get_theta.calculate_angle(img)
                    # else:
                    #     angle=90
                    # print(f"angle:{angle}")
                    # if angle!=0:
                        # with lock_serial: 
                    env.ser.write(b'\x66\x66\x01')
                    env.ser.write(angle.to_bytes(2, byteorder='big'))
                    # with lock_serial: 
                    command = b'\x66\x66\x06'
                    env.ser.write(command)
                    time.sleep(1.2)
                    command = b'\x66\x66\x0A'
                    env.ser.write(command)
                    # with lock_queue:
                    q_locate.put(b'\x66\x66\x98')
                    # print("put_command_to_q_locate")
                # ret, img = env.back_cap.read()
                
                # q_FT.put(b'\x66\x66\x05')
                #angle=0
            # time.sleep(5)

    # return angle
def run_policy(q_FT,q_STM,q_locate):
    while True: 
        if not q_STM.empty():
            char=q_STM.get()
            if char[0]==0x66 and char[1]== 0x66:
                if char[2]==0x85:
                    #q2.put(char)
                    with lock_queue:
                        q_FT.put(char)
                    print(f"收到数据_run_policy: {char.hex()}")
                    
                if char[2]==0x82 or char[2]==0x33:
                    #q3.put(char)
                    q_locate.put(char)
                    print(f"收到数据_run_policy: {char.hex()}")
                # elif char[2]==0x83 or char[2]==0x84:
                #     q_SGBM.put(char)
def agent_locate(env:MyEnv,q_locate,q_SGBM):
    front_cam=cv2.VideoCapture(front_cam_index)
    front_cam.set(cv2.CAP_PROP_EXPOSURE,front_light)
    turning_angle=3
    # gap_time=1
    ret,img=front_cam.read()
    cnt_reset=0
    if ret:
        img2=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        print("front_cam init")
    while True:
        if not q_locate.empty():
            char=q_locate.get()
            x=False
            y=False
            if char[0]==0x66 and char[1]== 0x66:
                # if char[2]==0x82:
                #     ret,frame=front_cam.read()
                #     while ret and not x and not y:
                #         angle,x,y=Apriltag.self_locate(frame)
                #         if angle and abs(angle)>turning_angle:
                #             env.com_car_rotate(angle)
                #         if x and y:
                #             env._agent_location=np.array([x,y])
                if char[2]==0x33:
                    env.reset()
#TODO with lock _com,init after if ret frame
                    ret,frame=front_cam.read()
                    if ret and not x and not y:
                        angle,x,y=Apriltag.self_locate(frame)
                        if angle and angle!=180 and abs(angle)>turning_angle:
                            print(f"目标角度: {angle}")
                            with lock:
                                # env.ser.write(b'\x66\x66\x09')
                                # env.com_car_rotate(angle)
                                # time.sleep(gap_time)
                                # env.ser.write(b'\x66\x66\x0A')

                                env.com_car_rotate(angle)
                        if angle==180:
                            env.ser.write(b'\x66\x66\x02')
                            env.ser.write(b'\x00\x00')
                            dis=abs(125-env._agent_location[0])
                            env.ser.write(dis.to_bytes(1, byteorder='big'))  
                        if x and y:
                            env._agent_location=np.array([x,y])
                    q_SGBM.put(char)
                    if cnt_reset==0:
                        env.com_move()
                        cnt_reset=1
                if char[2]==0x82:
#TODO with lock _com,init after if ret frame
                    ret,frame=front_cam.read()
                    if ret and not x and not y:
                        angle,x,y=Apriltag.self_locate(frame)
                        if angle and angle!=180 and abs(angle)>turning_angle:
                            print(f"目标角度: {angle}")
                            with lock:
                                # env.ser.write(b'\x66\x66\x09')
                                # env.com_car_rotate(angle)
                                # time.sleep(gap_time)
                                # env.ser.write(b'\x66\x66\x0A')

                                env.com_car_rotate(angle)
                        if angle==180:
                            env.ser.write(b'\x66\x66\x02')
                            env.ser.write(b'\x00\x00')
                            dis=abs(125-env._agent_location[0])
                            env.ser.write(dis.to_bytes(1, byteorder='big'))  
                        if x and y:
                            env._agent_location=np.array([x,y])
                    q_SGBM.put(char)
                if char[2]==0x97:
                    # print("get_char:0x97")
#TODO with lock _com,init after if ret frame
                    ret,frame=front_cam.read()
                    if ret and not x and not y:
                        angle,x,y=Apriltag.self_locate(frame)
                        if angle and angle!=180 and abs(angle)>turning_angle:
                            print(f"目标角度: {angle}")
                            with lock:
                                # env.ser.write(b'\x66\x66\x09')
                                # env.com_car_rotate(angle)
                                # time.sleep(gap_time)
                                # env.ser.write(b'\x66\x66\x0A')

                                env.com_car_rotate(angle)
                        if angle==180:
                            env.ser.write(b'\x66\x66\x02')
                            env.ser.write(b'\x00\x00')
                            dis=abs(125-env._agent_location[0])
                            env.ser.write(dis.to_bytes(1, byteorder='big'))  
                        if x and y:
                            env._agent_location=np.array([x,y])
                    if env.num_squares>0:
                        is_path=env.com_move()
                        if not is_path:
                            env.ser.write(b'\x66\x66\x02')
                            env.ser.write(b'\x00\x5A')
                            env.ser.write(b'\x14')
                    else:
                        env.ser.write(b'\x66\x66\x02')
                        env.ser.write(b'\x00\x5A')
                        env.ser.write(b'\x14')
                elif char[2]==0x99:
                    print("start fine_tuning")
                    while True:
                        if not q_locate.empty():
                            char=q_locate.get()
                            print(f"收到数据: {char.hex()}")
                            if char[0]==0x66 and char[1]== 0x66 and char[2]==0x98:
                                print("end fine_tuning")
                                char=b'\x66\x66\x97'
                                q_locate.put(char)
                                print(f"发送数据: {char.hex()}")
                                # env.com_move
                                break
                        else:
                            time.sleep(0.01)

def SGBM_init_2(env:MyEnv,q_SGBM,q_FT,q_left,q_right,q_back,q_back_2,q_left_2,q_right_2):
    cnt_SGBM=1
    repeat_time=9
    flag_b=0
    flag_l=0
    flag_r=0
    cnt_reset=0
    dis=30
    back_dis=150
    min_target_num=0
    while True:
        # break
        if not q_SGBM.empty():
            char=q_SGBM.get()
            if char[0]==0x66 and char[1]== 0x66 and char[2]==0x82:
                print("cnt_SGBM:",cnt_SGBM)
                print("repeat_time:",repeat_time)
                
                if cnt_SGBM%repeat_time==0 :
                    print("start SGBM_82")
                    
                    agent_location=env._agent_location
                    repeat_time=2
                    if agent_location[1]<200-dis and agent_location[0]>back_dis:
                        # char=q_SGBM.get()
                        q_left.put(b'\x66\x66\x89')
                        q_right.put(char)
                        q_back.put(char)
                    elif agent_location[1]>200+dis and agent_location[0]>back_dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(b'\x66\x66\x89')
                        q_back.put(char)
                    elif agent_location[0]<back_dis and agent_location[1]<200-dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(b'\x66\x66\x89')
                        q_back.put(b'\x66\x66\x89')
                    elif agent_location[0]<back_dis and agent_location[1]>200+dis:
                        # char=q_SGBM.get()
                        q_left.put(b'\x66\x66\x89')
                        q_right.put(char)
                        q_back.put(b'\x66\x66\x89')
                    elif agent_location[0]<back_dis and agent_location[1]>200-dis and agent_location[1]<200+dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(char)
                        q_back.put(b'\x66\x66\x89')
                    elif agent_location[0]>back_dis and agent_location[1]>200-dis and agent_location[1]<200+dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(b'\x66\x66\x89')
                        q_back.put(char)
                    cnt_SGBM+=1
                elif env.num_squares<min_target_num:
                    print("start SGBM_not_enough_target")
                    char=b'\x66\x66\x33'
                    agent_location=env._agent_location
                    # repeat_time=2``
                    if agent_location[1]<200-dis and agent_location[0]>back_dis:
                        # char=q_SGBM.get()
                        q_left.put(b'\x66\x66\x89')
                        q_right.put(char)
                        q_back.put(char)
                    elif agent_location[1]>200+dis and agent_location[0]>back_dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(b'\x66\x66\x89')
                        q_back.put(char)
                    elif agent_location[0]<back_dis and agent_location[1]<200-dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(b'\x66\x66\x89')
                        q_back.put(b'\x66\x66\x89')
                    elif agent_location[0]<back_dis and agent_location[1]>200+dis:
                        # char=q_SGBM.get()
                        q_left.put(b'\x66\x66\x89')
                        q_right.put(char)
                        q_back.put(b'\x66\x66\x89')
                    elif agent_location[0]<back_dis and agent_location[1]>200-dis and agent_location[1]<200+dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(char)
                        q_back.put(b'\x66\x66\x89')
                    elif agent_location[0]>back_dis and agent_location[1]>200-dis and agent_location[1]<200+dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(b'\x66\x66\x89')
                        q_back.put(char)
                    cnt_SGBM+=1
                    time.sleep(1)
                    # env.com_move()
                    q_FT.put(char)
                else:
                    print("times not enough")
                    q_FT.put(char)
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x33 and cnt_reset==0:
                cnt_reset=1
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x33 and cnt_reset==1:
                # if cnt_SGBM%repeat_time==0:
                if True:
                    print("start SGBM_33")
                    agent_location=env._agent_location
                    if agent_location[1]<200-dis and agent_location[0]>back_dis:
                        # char=q_SGBM.get()
                        q_left.put(b'\x66\x66\x89')
                        q_right.put(char)
                        q_back.put(char)
                    elif agent_location[1]>200+dis and agent_location[0]>back_dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(b'\x66\x66\x89')
                        q_back.put(char)
                    elif agent_location[0]<back_dis and agent_location[1]<200-dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(b'\x66\x66\x89')
                        q_back.put(b'\x66\x66\x89')
                    elif agent_location[0]<back_dis and agent_location[1]>200+dis:
                        # char=q_SGBM.get()
                        q_left.put(b'\x66\x66\x89')
                        q_right.put(char)
                        q_back.put(b'\x66\x66\x89')
                    elif agent_location[0]<back_dis and agent_location[1]>200-dis and agent_location[1]<200+dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(char)
                        q_back.put(b'\x66\x66\x89')
                    elif agent_location[0]>back_dis and agent_location[1]>200-dis and agent_location[1]<200+dis:
                        # char=q_SGBM.get()
                        q_left.put(char)
                        q_right.put(char)
                        q_back.put(char)
                cnt_SGBM+=1
        if not q_back_2.empty():
            char=q_back_2.get()
            if char[0]==0x66 and char[1]== 0x66 and char[2]==0x82:
                flag_b=1
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x33:
                flag_b=2
        if not q_left_2.empty():
            char=q_left_2.get()
            if char[0]==0x66 and char[1]== 0x66 and char[2]==0x82:
                flag_l=1
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x33:
                flag_l=2
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x89:
                flag_l=3
        if not q_right_2.empty():
            char=q_right_2.get()
            if char[0]==0x66 and char[1]== 0x66 and char[2]==0x82:
                flag_r=1
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x33:
                flag_r=2
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x89:
                flag_r=3
        if flag_b==2 and (flag_l==2 or flag_l==3) and (flag_r==2 or flag_r==3):
            print("SGBM_init done")
            if env.num_squares>0:
                is_path=env.com_move()
                if not is_path:
                    env.ser.write(b'\x66\x66\x02')
                    env.ser.write(b'\x00\x5A')
                    env.ser.write(b'\x14')
            else:
                env.ser.write(b'\x66\x66\x02')
                env.ser.write(b'\x00\x5A')
                env.ser.write(b'\x14')
            flag_b=0
            flag_l=0
            flag_r=0
        elif flag_b==1 and (flag_l==1 or flag_l==3)and (flag_r==1 or flag_r==3):
            print("get_photo_done")
            q_FT.put(b'\x66\x66\x82')
            flag_b=0
            flag_l=0
            flag_r=0

def SGBM_init(env:MyEnv,q_SGBM,q_FT,q_left,q_right,q_back,q_back_2,q_left_2,q_right_2):
    cnt=0
    flag_b=0
    flag_l=0
    flag_r=0
    while True:
        if not q_SGBM.empty():
            if cnt%4==0  or env.num_squares<3:
                # if agent-location[1]<150:
                char=q_SGBM.get()
                q_left.put(char)
                q_right.put(char)
                q_back.put(char)
            cnt+=1
        if not q_back_2.empty():
            char=q_back_2.get()
            if char[0]==0x66 and char[1]== 0x66 and char[2]==0x82:
                flag_b=1
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x33:
                flag_b=2
        if not q_left_2.empty():
            char=q_left_2.get()
            if char[0]==0x66 and char[1]== 0x66 and char[2]==0x82:
                flag_l=1
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x33:
                flag_l=2
        if not q_right_2.empty():
            char=q_right_2.get()
            if char[0]==0x66 and char[1]== 0x66 and char[2]==0x82:
                flag_r=1
            elif char[0]==0x66 and char[1]== 0x66 and char[2]==0x33:
                flag_r=2
        if flag_b==2 and flag_l==2 and flag_r==2:
            print("SGBM_init done")
            if env.num_squares>0:
                is_path=env.com_move()
                if not is_path:
                    env.ser.write(b'\x66\x66\x02')
                    env.ser.write(b'\x00\x5A')
                    env.ser.write(b'\x14')
            else:
                env.ser.write(b'\x66\x66\x02')
                env.ser.write(b'\x00\x5A')
                env.ser.write(b'\x14')
            flag_b=0
            flag_l=0
            flag_r=0
        elif flag_b==1 and flag_l==1 and flag_r==1:
            #!didn't FT after get_photo_done
            print("get_photo_done")
            q_FT.put(b'\x66\x66\x82')
            q_FT.put(b'\x66\x66\x85')
            flag_b=0
            flag_l=0
            flag_r=0

def SGBM_back_init(env:MyEnv,q_back,q_back_2):
    # left_camera_matrix = np.array([[6.658779254398679e+02,0,6.456685972506924e+02],
    # [0,6.654680419316929e+02,3.339317140939414e+02],
    #                             [0,0,1]])
    
    # right_camera_matrix = np.array([[6.666340134907508e+02,0,6.450309322183250e+02],
    # [0,6.662070938786943e+02,3.525276587772178e+02],
    # [0,0,1]])

    # left_distortion = np.array([[-0.053390802129390,0.127270321741189,5.196948472955012e-04,2.124273104283209e-04,-0.091559470679566]])
    
    # right_distortion = np.array([[-0.060693317140140,0.148720908337062,-4.696798593761847e-04,-3.517967377954150e-05,-0.109112699405528]])

    # R = np.array([[0.999380857975134,-0.030387164723515,0.017734738028798],
    #             [0.029695015211065,0.998833892144457,0.038066546667239],
    #             [-0.018870791835499,-0.037516344752927,0.999117819424657]])
    
    # T = np.array([1.501071158968365e+02,4.603268774394913,-3.230774963979015])
    back_left = np.array([[6.717365890549853e+02,0,6.481186047758437e+02],
    [0,6.703354014504876e+02,3.574636851067340e+02],
    [0,0,1]])
        
    back_right = np.array([[6.722822365415722e+02,0,6.496045771346123e+02],
    [0,6.710869258154073e+02,3.367679244703898e+02],
    [0,0,1]])

    back_left_distortion = np.array([[-0.047196231975254,0.046669902140363,6.508796688291871e-04,8.286496164414472e-04,0.126451544862522]])

    back_right_distortion = np.array([[-0.078056031604420,0.214683503709964,9.220621532501105e-04,0.001157623141003,-0.175733377269505]])

    R = np.array([[0.999165690975849,0.014100528580125,0.038328802101230],
    [-0.014587152141571,0.999816137297834,0.012446147646474],
    [-0.038146257603504,-0.012994871780831,0.999187668227669]])

    T = np.array([-80.889395516977190,0.529496587012572,-0.090861464763558])
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
    back_left_cam=cv2.VideoCapture(back_left_cam_index)
    back_right_cam=cv2.VideoCapture(back_right_cam_index)
    width=1280
    height=720
    back_left_cam.set(3, width)  # 设置宽度为640
    back_left_cam.set(4, height)  # 设置高度为480
    back_right_cam.set(3, width)  # 设置宽度为640
    back_right_cam.set(4, height)  # 设置高度为480    
    back_right_cam.set(cv2.CAP_PROP_EXPOSURE,back_right_light)  # back_right -6
    back_left_cam.set(cv2.CAP_PROP_EXPOSURE,back_left_light)  # back_left -5
    re_open_cnt=0     
    if back_left_cam.isOpened() and back_right_cam.isOpened():

        ret, frame1 = back_right_cam.read()
        ret2, frame2 = back_left_cam.read()
        if  ret and ret2 :
            imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            print("back_cam init success!")
    print("finish_calculate_back")
    while True:
        data=q_back.get()
        if data[0]==0x66 and data[1]==0x66:
            if data[2]==0x82:
                if back_left_cam.isOpened() and back_right_cam.isOpened() and re_open_cnt<7:
                    ret, frame1 = back_right_cam.read()
                    ret2, frame2 = back_left_cam.read()
                    agent_locate=env._agent_location
                    q_back_2.put(data)
                    if (ret and ret2):
                        # env._agent_location=np.array([Apriltag.self_locate(frame1)])
                        imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
                        imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                        img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
                        img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)
                        # img1_rectified = imgL
                        # imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
                        # imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)
                        disparity = stereo.compute(img1_rectified, img2_rectified)

                        # 归一化函数算法，生成深度图（灰度图）
                        #disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                        # 生成深度图（颜色图）
                        dis_color = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                        dis_color = cv2.applyColorMap(dis_color, 2)

                        # 计算三维坐标数据值
                        threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
                        # 计算出的threeD，需要乘以16，才等于现实中的距离
                        threeD = threeD * 16
                        boxes=SGBM_raspi.find_yellow_boxes(frame1)
                        targets=SGBM_raspi.cal_average_distance_in_boxes(boxes,threeD,1.05)
                        
                        env._get_obs(targets,agent_locate,camera_dir=0)
                        
                    else:
                        print("No frame!")
                        re_open_cnt+=1
                else:
                    re_open_cnt=0
                    print("No cam!")
                    back_left_cam.release()
                    back_right_cam.release()
                    back_left_cam = cv2.VideoCapture(back_left_cam_index) #r
                    back_right_cam = cv2.VideoCapture(back_right_cam_index)#l
                    width=1280
                    height=720
                    back_left_cam.set(3, width)  # 设置宽度为640
                    back_left_cam.set(4, height)  # 设置高度为480
                    back_right_cam.set(3, width)  # 设置宽度为640
                    back_right_cam.set(4, height)  # 设置高度为480   
                    back_right_cam.set(cv2.CAP_PROP_EXPOSURE,back_right_light)  # back_right -6
                    back_left_cam.set(cv2.CAP_PROP_EXPOSURE,back_left_light)  # back_left -5
            elif data[2]==0x33:
                if back_left_cam.isOpened() and back_right_cam.isOpened() and re_open_cnt<7:
                    ret, frame1 = back_right_cam.read()
                    ret2, frame2 = back_left_cam.read()
                    agent_locate=env._agent_location
                    #q_FT.put(data)
                    if (ret and ret2):
                        # env._agent_location=np.array([Apriltag.self_locate(frame1)])
                        imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
                        imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                        img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
                        img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)
                        # img1_rectified = imgL
                        # imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
                        # imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)
                        disparity = stereo.compute(img1_rectified, img2_rectified)

                        # 归一化函数算法，生成深度图（灰度图）
                        #disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                        # 生成深度图（颜色图）
                        dis_color = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                        dis_color = cv2.applyColorMap(dis_color, 2)

                        # 计算三维坐标数据值
                        threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
                        # 计算出的threeD，需要乘以16，才等于现实中的距离
                        threeD = threeD * 16
                        boxes=SGBM_raspi.find_yellow_boxes(frame1)
                        targets=SGBM_raspi.cal_average_distance_in_boxes(boxes,threeD,1.05)
                        
                        env._get_obs(targets,agent_locate,camera_dir=0)
                        q_back_2.put(data)
                        # env.com_move()


                    else:
                        print("No frame!")
                        re_open_cnt+=1
                else:
                    re_open_cnt=0
                    print("No cam!")
                    back_left_cam.release()
                    back_right_cam.release()
                    back_left_cam = cv2.VideoCapture(back_left_cam_index) #r
                    back_right_cam = cv2.VideoCapture(back_right_cam_index)#l
                    width=1280
                    height=720
                    back_left_cam.set(3, width)  # 设置宽度为640
                    back_left_cam.set(4, height)  # 设置高度为480
                    back_right_cam.set(3, width)  # 设置宽度为640
                    back_right_cam.set(4, height)  # 设置高度为480   
                    back_right_cam.set(cv2.CAP_PROP_EXPOSURE,back_right_light)  # back_right -6
                    back_left_cam.set(cv2.CAP_PROP_EXPOSURE,back_left_light)  # back_left -5


def SGBM_batteryside_init(env:MyEnv,q_right,q_right_2):
    # left_camera_matrix = np.array([[6.658779254398679e+02,0,6.456685972506924e+02],
    # [0,6.654680419316929e+02,3.339317140939414e+02],
    #                             [0,0,1]])
    
    # right_camera_matrix = np.array([[6.666340134907508e+02,0,6.450309322183250e+02],
    # [0,6.662070938786943e+02,3.525276587772178e+02],
    # [0,0,1]])

    # left_distortion = np.array([[-0.053390802129390,0.127270321741189,5.196948472955012e-04,2.124273104283209e-04,-0.091559470679566]])
    
    # right_distortion = np.array([[-0.060693317140140,0.148720908337062,-4.696798593761847e-04,-3.517967377954150e-05,-0.109112699405528]])

    # R = np.array([[0.999380857975134,-0.030387164723515,0.017734738028798],
    #             [0.029695015211065,0.998833892144457,0.038066546667239],
    #             [-0.018870791835499,-0.037516344752927,0.999117819424657]])
    
    # T = np.array([1.501071158968365e+02,4.603268774394913,-3.230774963979015])
    battery_left = np.array([[6.642806785950472e+02,0,6.458651529430067e+02],
    [0,6.637159214798321e+02,3.566890232056417e+02],
    [0,0,1]])
        
    battery_right = np.array([[6.651569191017091e+02,0,6.760976513986849e+02],
    [0,6.638472508330541e+02,3.452450815037580e+02],
    [0,0,1]])

    battery_left_distortion = np.array([[-0.059090205976626,0.183987698138405,-0.001474301322736,-8.965054064609814e-04,-0.152357391492578]])

    battery_right_distortion = np.array([[-0.049593992929082,0.137626633036958,0.001924420236102,6.303380661822436e-04,-0.116147283261452]])

    # R = np.array([[0.999986459881979,-0.004822102418715,-0.001956369333716],
    #               [0.004850026864318,0.999882682839651,0.014529204960947],
    #     [0.001886078503637,-0.014538496677622,0.999892531636392]])
    R = np.array([[0.999910913719758,-0.006443832864252,-0.011689381597682],
    [0.006542970023541,0.999942785050755,0.008462633645069],
    [0.011634180993507,-0.008538363013904,0.999895865672848]])
    T = np.array([-53.355887894759974,-0.025613407503569,-1.571592069253978])

    size=(1280,720)
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(battery_left, battery_left_distortion,
                                                                    battery_right, battery_right_distortion, size, R,
                                                                    T)
    left_map1, left_map2 = cv2.initUndistortRectifyMap(battery_left, battery_left_distortion, R1, P1, size, cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(battery_right, battery_right_distortion, R2, P2, size, cv2.CV_16SC2)
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
    battery_left_cam=cv2.VideoCapture(battery_left_cam_index)
    battery_right_cam=cv2.VideoCapture(battery_right_cam_index)
    width=1280
    height=720
    battery_left_cam.set(3, width)  # 设置宽度为640
    battery_left_cam.set(4, height)  # 设置高度为480
    battery_right_cam.set(3, width)  # 设置宽度为640
    battery_right_cam.set(4, height)  # 设置高度为480    
    battery_right_cam.set(cv2.CAP_PROP_EXPOSURE, battery_right_light)  #设置曝光值 1.0 - 5000  156.0
    battery_left_cam.set(cv2.CAP_PROP_EXPOSURE,battery_left_light)  #设置曝光值 1.0 - 5000  156.0
    re_open_cnt=0    
    if battery_left_cam.isOpened() and battery_right_cam.isOpened():
        ret, frame1 = battery_right_cam.read()
        ret2, frame2 = battery_left_cam.read()

        if  ret and ret2 :
            imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            print("battery_cam init success!")
    print("finish_calculate_battery")
    while True:
        data=q_right.get()
        if data[0]==0x66 and data[1]==0x66:
            if data[2]==0x82:
                if battery_left_cam.isOpened() and battery_right_cam.isOpened() and re_open_cnt<7:
                    ret, frame1 = battery_right_cam.read()
                    ret2, frame2 = battery_left_cam.read()
                    agent_locate=env._agent_location
                    q_right_2.put(data)
                    if (ret and ret2):
                        # env._agent_location=np.array([Apriltag.self_locate(frame1)])
                        imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
                        imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                        img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
                        img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)
                        # img1_rectified = imgL
                        # imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
                        # imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)
                        disparity = stereo.compute(img1_rectified, img2_rectified)

                        # 归一化函数算法，生成深度图（灰度图）
                        #disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                        # 生成深度图（颜色图）
                        dis_color = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                        dis_color = cv2.applyColorMap(dis_color, 2)

                        # 计算三维坐标数据值
                        threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
                        # 计算出的threeD，需要乘以16，才等于现实中的距离
                        threeD = threeD * 16
                        boxes=SGBM_raspi.find_yellow_boxes(frame1)
                        targets=SGBM_raspi.cal_average_distance_in_boxes(boxes,threeD,1)
                        
                        env._get_obs(targets,agent_locate,camera_dir=90)
                        
                    else:
                        print("No frame!")
                        re_open_cnt+=1
                else:
                    re_open_cnt=0
                    print("No cam!")
                    battery_left_cam.release()
                    battery_right_cam.release()
                    battery_left_cam = cv2.VideoCapture(battery_left_cam_index) #r
                    battery_right_cam = cv2.VideoCapture(battery_right_cam_index)#l
                    width=1280
                    height=720
                    battery_left_cam.set(3, width)  # 设置宽度为640
                    battery_left_cam.set(4, height)  # 设置高度为480
                    battery_right_cam.set(3, width)  # 设置宽度为640
                    battery_right_cam.set(4, height)  # 设置高度为480
                    battery_right_cam.set(cv2.CAP_PROP_EXPOSURE, battery_right_light)  #设置曝光值 1.0 - 5000  156.0
                    battery_left_cam.set(cv2.CAP_PROP_EXPOSURE, battery_left_light)  #设置曝光值 1.0 - 5000  156.0   
            elif data[2]==0x33:
                if battery_left_cam.isOpened() and battery_right_cam.isOpened() and re_open_cnt<7:
                    ret, frame1 = battery_right_cam.read()
                    ret2, frame2 = battery_left_cam.read()
                    agent_locate=env._agent_location
                    #q_FT.put(data)
                    if (ret and ret2):
                        # env._agent_location=np.array([Apriltag.self_locate(frame1)])
                        imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
                        imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                        img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
                        img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)
                        # img1_rectified = imgL
                        # imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
                        # imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)
                        disparity = stereo.compute(img1_rectified, img2_rectified)

                        # 归一化函数算法，生成深度图（灰度图）
                        #disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                        # 生成深度图（颜色图）
                        dis_color = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                        dis_color = cv2.applyColorMap(dis_color, 2)

                        # 计算三维坐标数据值
                        threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
                        # 计算出的threeD，需要乘以16，才等于现实中的距离
                        threeD = threeD * 16
                        boxes=SGBM_raspi.find_yellow_boxes(frame1)
                        targets=SGBM_raspi.cal_average_distance_in_boxes(boxes,threeD,theta=1)
                        # with lock:
                        env._get_obs(targets,agent_locate,camera_dir=90)
                        # env.com_move()
                        q_right_2.put(data)
                    else:
                        print("No frame!")
                        re_open_cnt+=1
                else:
                    re_open_cnt=0
                    print("No cam!")
                    battery_left_cam.release()
                    battery_right_cam.release()
                    battery_left_cam = cv2.VideoCapture(battery_left_cam_index) #r
                    battery_right_cam = cv2.VideoCapture(battery_right_cam_index)#l
                    width=1280
                    height=720
                    battery_left_cam.set(3, width)  # 设置宽度为640
                    battery_left_cam.set(4, height)  # 设置高度为480
                    battery_right_cam.set(3, width)  # 设置宽度为640
                    battery_right_cam.set(4, height)  # 设置高度为480
                    battery_right_cam.set(cv2.CAP_PROP_EXPOSURE, battery_right_light)  #设置曝光值 1.0 - 5000  156.0
                    battery_left_cam.set(cv2.CAP_PROP_EXPOSURE, battery_left_light)  #设置曝光值 1.0 - 5000  156.0   
            elif data[2]==0x89:
                q_right_2.put(data)
def SGBM_pumpside_init(env:MyEnv,q_left,q_left_2):
    # left_camera_matrix = np.array([[6.658779254398679e+02,0,6.456685972506924e+02],
    # [0,6.654680419316929e+02,3.339317140939414e+02],
    #                             [0,0,1]])
    
    # right_camera_matrix = np.array([[6.666340134907508e+02,0,6.450309322183250e+02],
    # [0,6.662070938786943e+02,3.525276587772178e+02],
    # [0,0,1]]) 

    # left_distortion = np.array([[-0.053390802129390,0.127270321741189,5.196948472955012e-04,2.124273104283209e-04,-0.091559470679566]])
    
    # right_distortion = np.array([[-0.060693317140140,0.148720908337062,-4.696798593761847e-04,-3.517967377954150e-05,-0.109112699405528]])

    # R = np.array([[0.999380857975134,-0.030387164723515,0.017734738028798],
    #             [0.029695015211065,0.998833892144457,0.038066546667239],
    #             [-0.018870791835499,-0.037516344752927,0.999117819424657]])
    
    # T = np.array([1.501071158968365e+02,4.603268774394913,-3.230774963979015])
    pump_left = np.array([[6.629714219465708e+02,0,6.552764493603507e+02],
                          [0,6.627627401785837e+02,3.673866482137107e+02],[0,0,1]])
        
    pump_right = np.array([[6.630597501042919e+02,0,6.470821199204139e+02],
    [0,6.626336469394710e+02,3.314077988629549e+02],
    [0,0,1]])

    pump_left_distortion = np.array([[-0.069666500520591,0.210081077069142,-1.857502891772009e-05,1.736149289216960e-04,-0.183842295110516]])

    pump_right_distortion = np.array([[-0.071516663523896,0.223341775166155,-1.832639845268067e-04,-0.001159739562814,-0.212903780057474]])

    R = np.array([[0.999970505713521,-7.602754114721856e-04,0.007642622870676],
    [6.621159550494266e-04,0.999917369978004,0.012838022384029],
                  [-0.007651751793331,-0.012832583433178,0.999888381519119]])

    T = np.array([-54.270925850730485,0.044042542371004,0.928487068566897])

    size=(1280,720)
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(pump_left, pump_left_distortion,
                                                                    pump_right, pump_right_distortion, size, R,
                                                                    T)
    left_map1, left_map2 = cv2.initUndistortRectifyMap(pump_left, pump_left_distortion, R1, P1, size, cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(pump_right, pump_right_distortion, R2, P2, size, cv2.CV_16SC2)
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
    pump_left_cam=cv2.VideoCapture(pump_left_cam_index)
    pump_right_cam=cv2.VideoCapture(pump_right_cam_index)
    width=1280
    height=720
    pump_left_cam.set(3, width)  # 设置宽度为640
    pump_left_cam.set(4, height)  # 设置高度为480
    pump_right_cam.set(3, width)  # 设置宽度为640
    pump_right_cam.set(4, height)  # 设置高度为480  
    pump_right_cam.set(cv2.CAP_PROP_EXPOSURE, pump_light)  #设置曝光值 1.0 - 5000  156.0
    pump_left_cam.set(cv2.CAP_PROP_EXPOSURE,pump_light)  #设置曝光值 1.0 - 5000  156.0  
    re_open_cnt=0     
    if pump_left_cam.isOpened() and pump_right_cam.isOpened() :

        ret, frame1 = pump_right_cam.read()
        ret2, frame2 = pump_left_cam.read()
        if  ret and ret2 :
            imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            print("pump_cam init success!")
    print("finish_calulate_pump")
    while True:
        data=q_left.get()
        if data[0]==0x66 and data[1]==0x66:
            if data[2]==0x82:
                if pump_left_cam.isOpened() and pump_right_cam.isOpened() and re_open_cnt<7:
                    ret, frame1 = pump_right_cam.read()
                    ret2, frame2 = pump_left_cam.read()
                    agent_locate=env._agent_location
                    q_left_2.put(data)
                    if (ret and ret2):
                        # env._agent_location=np.array([Apriltag.self_locate(frame1)])
                        imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
                        imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                        img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
                        img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)
                        # img1_rectified = imgL
                        # imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
                        # imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)
                        disparity = stereo.compute(img1_rectified, img2_rectified)

                        # 归一化函数算法，生成深度图（灰度图）
                        #disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                        # 生成深度图（颜色图）
                        dis_color = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                        dis_color = cv2.applyColorMap(dis_color, 2)

                        # 计算三维坐标数据值
                        threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
                        # 计算出的threeD，需要乘以16，才等于现实中的距离
                        threeD = threeD * 16
                        boxes=SGBM_raspi.find_yellow_boxes(frame1)
                        targets=SGBM_raspi.cal_average_distance_in_boxes(boxes,threeD,1)
                        # with lock:
                        env._get_obs(targets,agent_locate,camera_dir=270)
                        
                    else:
                        print("No frame!")
                        re_open_cnt+=1
                else:
                    re_open_cnt=0
                    print("No cam!")
                    pump_left_cam.release()
                    pump_right_cam.release()
                    pump_left_cam = cv2.VideoCapture(pump_left_cam_index) #r
                    pump_right_cam = cv2.VideoCapture(pump_right_cam_index)#l
                    width=1280
                    height=720
                    pump_left_cam.set(3, width)  # 设置宽度为640
                    pump_left_cam.set(4, height)  # 设置高度为480
                    pump_right_cam.set(3, width)  # 设置宽度为640
                    pump_right_cam.set(4, height)  # 设置高度为480
                    pump_right_cam.set(cv2.CAP_PROP_EXPOSURE, pump_light)  #设置曝光值 1.0 - 5000  156.0
                    pump_left_cam.set(cv2.CAP_PROP_EXPOSURE, pump_light)  #设置曝光值 1.0 - 5000  156.0     
            elif data[2]==0x33:
                if pump_left_cam.isOpened() and pump_right_cam.isOpened() and re_open_cnt<7:
                    ret, frame1 = pump_right_cam.read()
                    ret2, frame2 = pump_left_cam.read()
                    agent_locate=env._agent_location
                    #q_FT.put(data)
                    if (ret and ret2):
                        # env._agent_location=np.array([Apriltag.self_locate(frame1)])
                        imgL = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
                        imgR = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                        img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
                        img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)
                        # img1_rectified = imgL
                        # imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
                        # imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)
                        disparity = stereo.compute(img1_rectified, img2_rectified)

                        # 归一化函数算法，生成深度图（灰度图）
                        #disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                        # 生成深度图（颜色图）
                        dis_color = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                        dis_color = cv2.applyColorMap(dis_color, 2)
                        
                        # 计算三维坐标数据值
                        threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
                        # 计算出的threeD，需要乘以16，才等于现实中的距离
                        threeD = threeD * 16
                        boxes=SGBM_raspi.find_yellow_boxes(frame1)
                        targets=SGBM_raspi.cal_average_distance_in_boxes(boxes,threeD,1)
                        # with lock:
                        env._get_obs(targets,agent_locate,camera_dir=270)
                        # env.com_move()
                        q_left_2.put(data)
                    else:
                        print("No frame!")
                        re_open_cnt+=1
                else:
                    re_open_cnt=0
                    print("No cam!")
                    pump_left_cam.release()
                    pump_right_cam.release()
                    pump_left_cam = cv2.VideoCapture(pump_left_cam_index) #r
                    pump_right_cam = cv2.VideoCapture(pump_right_cam_index)#l
                    width=1280
                    height=720
                    pump_left_cam.set(3, width)  # 设置宽度为640
                    pump_left_cam.set(4, height)  # 设置高度为480
                    pump_right_cam.set(3, width)  # 设置宽度为640
                    pump_right_cam.set(4, height)  # 设置高度为480   
                    pump_right_cam.set(cv2.CAP_PROP_EXPOSURE, pump_light)  #设置曝光值 1.0 - 5000  156.0
                    pump_left_cam.set(cv2.CAP_PROP_EXPOSURE, pump_light)  #设置曝光值 1.0 - 5000  156.0  
            elif data[2]==0x89:
                q_left_2.put(data)
def com_to_STM(env:MyEnv,q):
    while True:
    # 读取3个字节的数据
        with lock_serial:
            data = env.ser.readline()
            if data:
                print("success receive data:",data)
                q.put(data)
        time.sleep(0.2)
#python3 /home/simon/Desktop/src/policy.py
if __name__=="__main__":
        print("start")
        env=MyEnv()
        # env.cam_init()
        # ret,img=env.right_cap.read()
        # env._agent_location=np.array([Apriltag.self_locate(img)])
        #env.com_move
        q_SGBM = queue.Queue()
        q_STM= queue.Queue()
        q_locate=queue.Queue()
        q_FT= queue.Queue()
        q_back=queue.Queue()
        q_right=queue.Queue()
        q_left=queue.Queue()
        q_left_2=queue.Queue()
        q_right_2=queue.Queue()
        q_back_2=queue.Queue()
        t1 = threading.Thread(target=SGBM_init_2, args=(env,q_SGBM,q_FT,q_left,q_right,q_back,q_back_2,q_left_2,q_right_2))
        t2 = threading.Thread(target=run_policy, args=(q_FT,q_STM,q_locate,))
        t3 = threading.Thread(target=com_to_STM, args=(env,q_STM,))
        t4=threading.Thread(target=fine_tuning,args=(env,q_FT,q_locate,))
        t5=threading.Thread(target=agent_locate,args=(env,q_locate,q_SGBM,))
        t6=threading.Thread(target=SGBM_back_init,args=(env,q_back,q_back_2,))
        t7=threading.Thread(target=SGBM_batteryside_init,args=(env,q_right,q_right_2,))
        t8=threading.Thread(target=SGBM_pumpside_init,args=(env,q_left,q_left_2,))
        t1.start()
        t2.start()
        t3.start()
        t4.start()
        t5.start()
        t6.start()
        # time.sleep(5)
        t7.start()
        t8.start()
        t1.join()
        t2.join()
        t3.join()
        t4.join()
        t5.join()
        t6.join()   
        t7.join()
        t8.join()
        env.close()
        # env.reset()
        #env._get_obs()
        # env.recieve_data_myversion()
        # angle=get_theta.read_angle(env)
        # env.ser.write(b'\x66\x66\x01')
        # env.ser.write(angle.to_bytes(2, byteorder='big'))
        # env.ser.write(b'\x66\x66\x06')

