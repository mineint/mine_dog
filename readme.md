


# 前期准备

1. 进入项目目录下的can文件夹
```bash
cd USB2CAN-Demo-Lingzu/can
```
2. 复制规则文件usb_can.rules 到/etc/udev/rules.d/
```bash
sudo cp usb_can.rules /etc/udev/rules.d/
```
3. 运行下面的命令，使udev规则生效
```bash
sudo udevadm trigger
```

编译流程
```bash
mkdir build
```
```bash
cmake -S . -B build
```
```bash
cd build
```
```bash
make
```
```bash
./can_node
```
mkdir build
cmake -S . -B build
cd build
make

sudo usermod -a -G dialout $USER

关键步骤： 执行完后，你必须注销当前用户并重新登录（或者直接重启电脑），修改才会生效。

重新登录后，输入 groups 命令，如果你能在列表中看到 dialout，说明权限已获得。此时直接运行你的程序即可。


先确认你的设备路径（假设是 /dev/ttyUSB0）。

执行：
code Bash

sudo chmod 666 /dev/ttyUSB0

注：666 表示允许所有用户读写该设备。

缺点： 一旦你拔掉 USB 再插上，或者重启电脑，权限会重置，你得重新运行这个命令。