# 导入 socket、sys 模块
import socket
import sys
import os

import string
import csv
import numpy as np
import matplotlib.pyplot as plt


cwd = os.getcwd() 
files = os.listdir(cwd)

#csvFile = open("HDmap_whole_lane.csv", "r")
csvFile = open("C:\\Users\Administrator\Desktop\Route_Process\HDmap_whole_lane.csv", "r")
map_info_org = list(csv.reader(csvFile))  # convert the data into a list type

#Process Whole Lane
#map_info= np.array(list(map_info_org))

map_length = len(map_info_org)

lanes = np.zeros([4,map_length,2], dtype=float)

for i in range(map_length): 
    for ii in range(4):
        n=ii*3+1
        lanes[ii,i,0] = map_info_org[i][n]
        lanes[ii,i,1] = map_info_org[i][n+1]

#print(lanes)
density = 10

plt.figure(1)
plt.subplot(211) # plot the map
for ii in range(4):
    for i in range(int(map_length/density)):
        plt.plot(lanes[ii,i*density,0],lanes[ii,i*density,1],'b.')

#######################################################################
# 创建 socket 对象
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 
# 获取本地主机名
#host = socket.gethostname()
host = '10.75.81.226'

# 设置端口好
port = 27015
 
# 连接服务，指定主机和端口
s.connect((host, port))
sendmsg='$FMWSM,1,-25,-310,0,0,0,&*'
 
# 接收小于 1024 字节的数据

plt.ion() # interactive mode on 

for i in range(500):
        
        msg2 = []
        msg = s.recv(50000)
        msg2=msg.decode('utf-8')
        
        #### Process Data ####
        msg_line = msg2.split('\n')
        x = []
        y = []
        v = []

        for i in range(len(msg_line)-1):
            msg_single_line = msg_line[i]
            block = msg_single_line.split(',')
            x.append(block[0])
            y.append(block[1])
            v.append(block[5])
            
        yy = list(map(float, y))
        xx = list(map(float, x))
        vv = list(map(float, v))
            
        plt.figure(1)
        plt.subplot(211) # plot the trajectory
        #pt.clf() 
        plt.plot(xx, yy)
        x_min = min(xx)-10
        x_max = max(xx)+10
        y_min = min(yy)-10
        y_max = max(yy)+10 
        plt.axis([x_min,x_max,y_min,y_max])
        #plt.show()

        
        plt.figure(1)
        #plt.clf() 
        plt.subplot(224)
        plt.axis('off')
        i = len(v)
        plt.plot(range(i), vv)    
        
        
        ######################
        s.send(sendmsg.encode())
        plt.draw()
        plt.pause(1e-10)
 
s.close()