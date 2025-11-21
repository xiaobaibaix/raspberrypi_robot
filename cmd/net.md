# 通讯
# ROS_DOMAIN_ID=1 双方id==1
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc

#   共享文件夹
sudo vim /etc/fstab
.host:/ /mnt/hgfs fuse.vmhgfs-fuse allow_other,uid=1000,gid=1000,umask=022 0 0
