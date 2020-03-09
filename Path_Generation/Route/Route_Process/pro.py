# 导入 socket、sys 模块
import socket
import sys
import string
import numpy as np
import matplotlib.pyplot as plt

#bolin Zhao. bolin.zhao@ff.com

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
        plt.clf() 
        plt.axis('off')
        plt.plot(xx, yy)

        plt.figure(2)
        plt.clf() 
        plt.axis('off')
        i = len(v)
        plt.plot(range(i), vv)    
        
        
        ######################
        s.send(sendmsg.encode())
        plt.draw()
        plt.pause(1e-10)
 
s.close()