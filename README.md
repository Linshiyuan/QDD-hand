# QDD-hand
## 准备工作

 ### 安装can-utils
```bash
sudo apt install can-utils

# 打开can口 
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0


# 测试can口是否正常

# 显示接收信息
candump can0 

# 生成随机can帧发送
cangen can0 -g 0 -i -x
```
 ### 运行程序
```bash
# 打开can口  插上模块后仅需运行一次
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
# 运行程序
source devel/setup.bash    

rosrun robotis_control canTest
```