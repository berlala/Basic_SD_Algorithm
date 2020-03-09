
#!/usr/bin/python3
# 文件名：client.py
 
# 导入 socket、sys 模块
import socket
import sys
import string
#读入地图文件
i=1
print(i)
i=i+1
print(i)
 
# 创建 socket 对象
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 
# 获取本地主机名
host = socket.gethostname()
 
# 设置端口好
port = 27015
 
# 连接服务，指定主机和端口
s.connect((host, port))
sendmsg='$FMWSM,1,-25,-310,0,0,0,&*'
 
# 接收小于 1024 字节的数据
while 1:
          msg = s.recv(50000)
          msg2=msg.decode('utf-8')
          
          print (msg2)

          s.send(sendmsg.encode())
 
s.close()
 
