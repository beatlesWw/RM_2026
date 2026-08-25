
#开机记得启动#

export VCPKG_ROOT=~/vcpkg

export PATH=$VCPKG_ROOT:$PATH

source /home/hero/intel/openvino_2024/setupvars.sh

运行demo:
./build/auto_aim_test

需要运行：
/home/hero/xuezhang/auto_aim_refactoring-master/io/galaxy

其中：
include：头文件

针对命令有问题：
./build/camera_test: error while loading shared libraries: liblog4cplus_gx.so: cannot open shared object file: No such file or directory

是路径问题，所以应该再次加上：
 export LD_LIBRARY_PATH=/home/hero/xuezhang/auto_aim_refactoring-master/auto_aim_refactoring/io/galaxy/lib/x86_64:$LD_LIBRARY_PATH

(找连接：
https://flowus.cn/share/dd80b179-864d-452f-bac6-b4f5c4c20360?code=SZFTW9
【FlowUs 息流】xuezhang的里面的动态库问题
)

11.4新增
sudo ip link set up vcan0 //启动虚拟can接口



总：
export VCPKG_ROOT=~/vcpkg
export PATH=$VCPKG_ROOT:$PATH
source /home/hero/intel/openvino_2024/setupvars.sh
export LD_LIBRARY_PATH=/home/hero/xuezhang/auto_aim_refactoring-master/io/galaxy/lib/x86_64:$LD_LIBRARY_PATH
第四个原来（但是现在用不了）：export LD_LIBRARY_PATH=/home/hero/xuezhang/auto_aim_refactoring-master/auto_aim_refactoring/io/galaxy/lib/x86_64:$LD_LIBRARY_PATH
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan   //重启的话这两条需要写一下
sudo ip link set up vcan0 
//启动虚拟can接口


//虚拟的串口
sudo socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1,mode=666
sudo chmod 666 /tmp/ttyV0

