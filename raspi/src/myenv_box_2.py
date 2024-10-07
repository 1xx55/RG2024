import struct
import gymnasium as gym
from gymnasium import spaces
import pygame
import numpy as np
import time 
import random
import cv2
import math
import heapq
import serial
import get_theta
camera_direction =0
size_env = (300,400)
target_len =15
agent_len =60
num_add_target=10

class MyEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}
    init_agent_locat=[150,342]
    #TODO get the init location of target
    init_target_locat=([107,28],[147,28],[187,28],[28,129],[28,169],[28,209]
                       ,[28,249],[265,118],[265,158],[265,198])
    def __init__(self, size=5):
        pygame.init()
        self.serialinit()
        self.window = pygame.display.set_mode((300, 400))
        self.size = size  # The size of the square grid
        self.window_size = 400  # The size of the PyGame window
        self.reset()
        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        self.observation_space = spaces.Dict(
            {
                "agent": spaces.Box(0, size - 1, shape=(2,), dtype=float),
                "target": spaces.Box(0, size - 1, shape=(2,), dtype=float),
            }
        )
        #! spaces.Box(0, [width,length], shape=(2,), dtype=float)
        self._agent_length=agent_len
        # self._agent_location = self.np_random.integers(0, self.window_size-self._agent_length
        #                                                , size=2, dtype=int)

        self.camera_dir=0
        self.catch_dir=0
        """
        If human-rendering is used, `self.window` will be a reference
        to the window that we draw to. `self.clock` will be a clock that is used
        to ensure that the environment is rendered at the correct framerate in
        human-mode. They will remain `None` until human-mode is used for the
        first time.
        """
        self.window = None
        self.clock = None
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
                    if target[0] > max_x or (target[0] == max_x and target[1] > max_y):
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
                return []
            #todo: determine target location
            index.append(i)
            map=self.make_map()
            if(self.catch_dir==0):
                path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]+target_len/2+agent_len/2)+1,int(target[1])])
            if(self.catch_dir==1):
                path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]),int(target[1]+target_len/2+agent_len/2)+1])
            if(self.catch_dir==2):
                path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]),int(target[1]-target_len/2-agent_len/2)-1])
            if(self.catch_dir==3):
                path = self.astar(array=map, start=self._agent_location, goal=[int(target[0]-target_len/2-agent_len/2)-1,int(target[1])])
            
            
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
            #delete target
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
        return path
    # def find_nearest_target(self):
    #     distances = [self.euclidean_distance(self._agent_location, target) for target in self._target_location]
    #     nearest_target_index = np.argmin(distances)
    #     return self._target_location[nearest_target_index],nearest_target_index
    
    def change_camera_dir(self,angle):
        #TODO commute to stm to change
        self.com_cam_rotate(angle)
        self.camera_dir=angle

    # def camera_dir(self):
    #     self.camera_dir = camera_direction
    def remove_obj_in_scope(self):
        for target in self._target_location:
            delta=target-self._agent_location
            if delta[0]==0:
                if delta[1]>0:
                    theta=270
                else:
                    theta=90
                continue
            else:
                theta=math.atan(abs(delta[1])/abs(delta[0]))
                if delta[1]<0 and delta[0]<0:
                    theta=theta+90
                elif delta[1]>0 and delta[0]<0:
                    theta=theta+180
                elif delta[1]>0 and delta[0]>0:
                    theta=theta+270
            theta_scope=math.atan(65/80)
            if theta<(self.camera_dir+theta_scope)%360 and theta> (360+(self.camera_dir-theta_scope))%360:
                index = np.where((self._target_location == target).all(axis=1))
                self._target_location = np.delete(self._target_location, index, axis=0)
        return {"agent": self._agent_location, "target": self._target_location}
        #a=fun(self) print(a["agent"])
    
    def _get_obs(self):
        self.remove_obj_in_scope()

        #todo relocate the pos of target in scope(check the scope first)
        #self.add_certain_target(target_loca)
        #!the format of target_loca: ((,),(,))
        #get the direction of camera
        return {"agent": self._agent_location, "target": self._target_location}

    def _get_info(self):
        #todo get the pos of robot
        # return {
        #     "distance": np.linalg.norm(
        #         self._agent_location - self._target_location[0], ord=2
        #     ) # get norm(=distance while ord=2)
        # }
        return self._agent_location
    def reset(self, seed=None, return_info=False, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)

        # Choose the agent's location uniformly at random
        self._agent_length=agent_len
        # self._agent_location = self.np_random.integers(0, self.window_size-self._agent_length
        #                                                , size=2, dtype=int)
        self._agent_location = np.array(self.init_agent_locat)


        # self._target_location = self._agent_location
        # while np.array_equal(self._target_location, self._agent_location):
        #     self._target_location = self.np_random.integers(0, self.size, size=2, dtype=int)
        self.init_target()
        self.camera_dir=0
        observation = self._get_obs()
        info = self._get_info()
        return (observation, info) if return_info else observation
    def init_target(self):
        self._target_length=target_len
        # self._target_location = np.random.randint(0, self.window_size-self._target_length
        #                                            , size=(num_squares, 2))
        self._target_location = np.array(self.init_target_locat)
        self.num_squares=len(self.init_target_locat)
        # 为了确保y坐标小于400，我们需要单独处理y坐标
        return(self._target_length,self.num_squares,self._target_location)
    def add_certain_target(self,target_loca):
        new_target1 = np.array(target_loca)
        self.num_squares=self.num_squares+len(target_loca)
# 将新目标添加到目标位置数组中
        self.target_locations = np.append(self.target_locations, new_target1, axis=0)
        # self._target_location = np.random.randint(0, self.window_size-self._target_length
        #                                            , size=(num_squares, 2))
        return(self._target_length,self.num_squares,self._target_location,self._target_angles)
    def add_ran_target(self,num_squares):
        self._target_length=target_len
        # self._target_location = np.random.randint(0, self.window_size-self._target_length
        #                                            , size=(num_squares, 2))
        new_target = np.random.randint(low=0, high=300-15, size=(num_squares, 2))

        # 为了确保y坐标小于400，我们需要单独处理y坐标
        new_target[:, 1] = np.random.randint(low=0, high=400-15, size=10)
        self._target_location = np.append(self._target_location, new_target, axis=0)
        self._target_angles = np.random.randint(0, 90, size=num_squares)
        self.num_squares=self.num_squares+num_squares
        return(self._target_length,self.num_squares,self._target_location,self._target_angles)
    
    # 计算旋转后的正方形顶点坐标
    def rotate_point(self, point, angle, center):
        x, y = point
        cx, cy = center
        rad = math.radians(angle)
        new_x = cx + math.cos(rad) * (x - cx) - math.sin(rad) * (y - cy)
        new_y = cy + math.sin(rad) * (x - cx) + math.cos(rad) * (y - cy)
        return [new_x, new_y]
    # 绘制正方形
    def draw_square(self, screen, pos, size, color, angle):
        pygame.draw.polygon(screen, color, [
            self.rotate_point([pos[0] - size / 2, pos[1] - size / 2], angle, pos),
            self.rotate_point([pos[0] + size / 2, pos[1] - size / 2], angle, pos),
            self.rotate_point([pos[0] + size / 2, pos[1] + size / 2], angle, pos),
            self.rotate_point([pos[0] - size / 2, pos[1] + size / 2], angle, pos)
        ])
    


    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        direction = action
        # We use `np.clip` to make sure we don't leave the grid
        self._agent_location = np.clip(
            self._agent_location + direction, 0, self.size 
        )
        #*clip restricts the range to 0 -- size-1
        # An episode is done if the agent has reached the target
        done = np.array_equal(self._agent_location, self._target_location)
        reward = 1 if done else 0  # Binary sparse rewards
        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, done, info
    def render_agent_move(self, path, mode="human"):
        pygame.init()
        self.window = pygame.display.set_mode((300, 400))
        canvas = pygame.Surface(size_env)
        canvas.fill((255, 255, 255))
        #draw target
        for i in range(self.num_squares):
            # self.draw_square(canvas, self._target_location[i], self._target_length
            #                   ,(0, 0, 0),self._target_angles[i])
            self.draw_square(canvas, self._target_location[i], self._target_length
                              ,(0, 0, 0),0)
            # Now we draw the agent
        self.draw_square(canvas, self._agent_location, self._agent_length
                              ,(0, 0, 255),0)
        #draw agent move
        for i in range(len(path)-1):
            pygame.draw.line(canvas, (0, 0, 255), path[i], path[i+1], 2)
        # The following line copies our drawings from `canvas` to the visible window
        if mode == "human":
            # The following line copies our drawings from `canvas` to the visible window
            self.window.blit(canvas, canvas.get_rect())
            pygame.event.pump()
            pygame.display.update()

            # We need to ensure that human-rendering occurs at the predefined framerate.
            # The following line will automatically add a delay to keep the framerate stable.
            # self.clock.tick(self.metadata["render_fps"])
        else:  # rgb_array
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(canvas)), axes=(1, 0, 2)
            )
        pygame.display.flip()
        # We need to ensure that human-rendering occurs at the predefined framerate.
        # The following line will automatically add a delay to keep the framerate stable.
        # self.clock.tick(self.metadata["render_fps"])
    def render(self, mode="human"):
        if self.window is None and mode == "human":
            pygame.init()
            pygame.display.init()
            self.window = pygame.display.set_mode((self.window_size, self.window_size))
        if self.clock is None and mode == "human":
            self.clock = pygame.time.Clock()

        canvas = pygame.Surface(size_env)
        canvas.fill((255, 255, 255))

        #draw target
        for i in range(self.num_squares):
            # self.draw_square(canvas, self._target_location[i], self._target_length
            #                   ,(0, 0, 0),self._target_angles[i])
            self.draw_square(canvas, self._target_location[i], self._target_length
                              ,(0, 0, 0),0)
            # Now we draw the agent
        self.draw_square(canvas, self._agent_location, self._agent_length
                              ,(0, 0, 255),0)

        if mode == "human":
            # The following line copies our drawings from `canvas` to the visible window
            self.window.blit(canvas, canvas.get_rect())
            pygame.event.pump()
            pygame.display.update()
            # We need to ensure that human-rendering occurs at the predefined framerate.
            # The following line will automatically add a delay to keep the framerate stable.
            # self.clock.tick(self.metadata["render_fps"])
        else:  # rgb_array
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(canvas)), axes=(1, 0, 2)
            )
        #close window
        if self.window is not None and mode == "human":
            pygame.display.quit()
            pygame.quit()

    def close(self):
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()

            #!change the format of ser
    def serialinit(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
        except:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.5)
        self.ser.bytesize = 8
        return self.ser
    def com_cam_rotate(self,angle):
        self.ser.write(b'\x66\x66\x01')
        #TODO angle is verse clockwise
    def com_move(self):
        path=self.A_star_path_planning()
        for i in range(len(path)-1):
            self.ser.write(b'\x66\x66\x02')
            if path[i][1]-path[i+1][1] < 0:
                dis=path[i][1]-path[i+1][1]
                self.ser.write(b'\x00\x5A')
                self.ser.write(dis.to_bytes(1, byteorder='big'))
            elif path[i][1]-path[i+1][1] > 0:
                dis=path[i+1][1]-path[i][1]
                self.ser.write(b'\x01\x0E')
                self.ser.write(dis.to_bytes(1, byteorder='big'))

            elif path[i][0]-path[i+1][0] < 0:
                dis=path[i][0]-path[i+1][0]
                self.ser.write(b'\x00\xB4')
                self.ser.write(dis.to_bytes(1, byteorder='big'))

            elif path[i][0]-path[i+1][0] > 0:
                dis=path[i+1][0]-path[i][0]
                self.ser.write(b'\x00\x00')
                self.ser.write(dis.to_bytes(1, byteorder='big'))
        if self.catch_dir==1:
            self.com_car_rotate(90)
            self.com_car_rotate(270)
        elif self.catch_dir==2:
            self.com_car_rotate(270)
            self.com_car_rotate(90)
        elif self.catch_dir==3: 
            self.com_car_rotate(180)
            self.com_car_rotate(180)
    def com_car_rotate(self,angle):
        self.ser.write(b'\x66\x66\x03')
        #*angle is verse clockwise
        self.ser.write(angle.to_bytes(2, byteorder='big'))
    def recieve_data(self):
        while True:
    # 读取3个字节的数据
            data = self.serial.readline()
            if data:
                # 打印接收到的数据
                print(data)
                # 检查前三个字节是否为0x66
                if data[0] == 0x66 and data[1] == 0x66 :
                    # 发送两个字节的数据0x66
                    if data[2] == 0x81:
                        angle=get_theta.read_angle ()
                        message=int(38/9*angle+300)
                        self.ser.write(b'\x66\x66\x01')
                        self.ser.write(message.to_bytes(2, byteorder='big'))
                        self.ser.write(b'\x66\x66\x05')
                    elif data[2] == 0x82:
                        #*stm has finished movement
                        pass
                    elif data[2] == 0x83:
                        #! do not need
                        #*stm has finished shooting
                        self.com_move()
                        
    #TODO: 增加对stm的控制射速
    def recieve_data_myversion(self):
        while True:
    # 读取3个字节的数据
            data = self.serial.readline()
            if data:
                # 打印接收到的数据
                print(data)
                # 检查前三个字节是否为0x66
                if data[0] == 0x66 and data[1] == 0x66 :
                    # 发送两个字节的数据0x66
                    if data[2] == 0x81:
                        angle=get_theta.read_angle ()
                        self.ser.write(b'\x66\x66\x02')
                        self.ser.write(angle.to_bytes(2, byteorder='big'))
                        self.com_move()
                        
if __name__=="__main__":
    env=MyEnv()
    env.__init__()
    env.reset()
    # env.add_ran_target(10)
    # env.render()
    while(env.num_squares!=0):
        path=env.A_star_path_planning()
        env.render_agent_move(path)
        time.sleep(0.5)
    # while not np.array_equal(env._target_location, env._agent_location):
    #     env.render()
    #     env.step(env.random_action())
    env.close()
# if __name__=="__main__":
#     env=MyEnv()
#     env.__init__()
#     env.reset()
#     env.com_move()
#     env.recieve_data_myversion()
