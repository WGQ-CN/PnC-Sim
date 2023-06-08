### Gazebo无法打开（卡在启动界面）

**解决方法：**手动下载模型，给予访问权限

```shell
cd ~/.gazebo/
git clone https://github.com/osrf/gazebo_models.git models

sudo chmod 777 ~/.gazebo/models
sudo chmod 777 ~/.gazebo/models/*
```

将此文件夹整个复制到`~/.gazebo/models`，此文件夹为地图模型文件。

