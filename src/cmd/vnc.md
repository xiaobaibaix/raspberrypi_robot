# vnc

vncserver -kill :3	//删掉vncserver5903端口监听
sudo systemctl status vncserver@:1.service //查看状态

sudo rm /etc/systemd/system/vncserver@:3.service    //删除服务

sudo systemctl daemon-reload    //重新加载服务

vncserver -geometry 1280x720 -depth 16 :1 -localhost no //启动vnc服务器


minicom -D /dev/ttyAMA0 -b 115200