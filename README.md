# FR5 + ZED X 眼在手上标定

这里的脚本使用 FAIRINO Python SDK 读取 FR5 **法兰**位姿，用 ZED SDK 保存 **ZED X 左目已校正图像**和实际内参。标定板固定在工作空间，机器人携相机移动。求解先比较 OpenCV Park–Martin 和 Daniilidis，再以 Park 为初值进行全角点像素重投影优化；结果由留出姿态的重投影误差选择。

## 开始前

1. 在能使用 ZED SDK 的 Python 环境中安装 `requirements.txt`（OpenCV 限定 4.x，因为 [5.0.0.93 的 Python 包缺失 `calibrateHandEye`](https://github.com/opencv/opencv/issues/29565)）。另外按厂商说明安装 [ZED SDK Python API](https://www.stereolabs.com/docs/app-development/python/install) 和 [FAIRINO Python SDK](https://github.com/FAIR-INNOVATION/fairino-python-sdk)。确认 Python 版本受本机 ZED SDK 支持。
2. [target.example.yaml](target.example.yaml) 根据照片文字填写了 6×6、tagSize `0.055 m`、tagSpacing `0.3`（间隔 `0.0165 m`）。这是**名义尺寸**；收到实际 YAML 后改用它，并测量打印后实体 tag 边长。`tagSpacing` 是间隔与边长之比，不是米。
3. ZED X 是 GMSL2 相机，直接采集需 Jetson + ZED Link/ZED Box；Windows 端可用 [ZED SDK 流接收](https://docs.stereolabs.com/docs/products/cameras/zedx/development-on-pc)，但 Jetson 端需先运行 [官方发送程序](https://github.com/stereolabs/zed-sdk/blob/master/camera%20streaming/single_sender/python/streaming_sender.py)。`--stream-ip` 接收流；不填则打开本机直连相机。
4. 将标定板牢固固定。采集时手动移动机器人，脚本不会发运动指令。建议采集 20–30 个姿态，改变绕至少两个不同轴的转角，并让标定板覆盖图像不同区域。每次按 Enter 前等待机械臂稳定；脚本会比较抓图前后的法兰位姿，运动超限即丢弃该帧。

## 使用

先用标定板照片确认标签编号和角点方向：

```powershell
python handeye.py detect --image board_photo.jpg --target target.example.yaml --overlay detected.png
```

Kalibr 原版 6×6 板的 tag 0 在左下，5 在右下，30 在左上，35 在右上；每个 tag 的蓝色 `0` 应在该 tag 的**物理右下角**。先前的局部照片检测出 tag 0、6；新提供的整板照片检测出 0–35 全部 36 个标签，角点布局与当前代码一致。可用上方 `detect --overlay` 命令在本地生成编号覆盖图；诊断照片不随源码提交。若检测编号或角点方向不同，先检查板的制作方式，不能直接套用当前几何映射。

在 Jetson 直连相机时采集：

```powershell
python handeye.py capture --robot-ip 192.168.58.2 --target target.example.yaml --output session_001
```

在接收 ZED SDK 流的主机上采集：

```powershell
python handeye.py capture --robot-ip 192.168.58.2 --stream-ip 192.168.1.10 --target target.example.yaml --output session_001
```

`192.168.58.2` 只是法奥文档中的示例地址，运行时替换为控制器实际 IP；流地址替换为 Jetson 的 IP。`--output` 必须是尚不存在的目录。程序保存 `session.json`、`target.yaml` 和 `images/*.png`。ZED SDK 启动时可能更新自标定内参，因此每个 session 都保存当次左目内参。`VIEW.LEFT` 是已校正图，脚本只配合 `calibration_parameters.left_cam` 使用，不依赖深度图或名义 2.2 mm 焦距。

求解：

```powershell
python handeye.py solve --session session_001
```

如果之后测得正确 YAML，求解时可指定 `--target measured_target.yaml`。`result.json` 中 `flange_T_left_camera` 是将左目光学坐标变换到法兰坐标的 4×4 矩阵，平移单位米；`left_camera_T_flange` 是逆矩阵。`base_T_board` 只用于固定板一致性检查。报告同时列出每个算法的训练和留出姿态误差、拒绝样本与 PnP 误差。至少需要 12 个可用姿态，建议更多。

模拟测试：

```powershell
python -m unittest discover -s tests -v
```

## 坐标和可靠性核对

- [FAIRINO 手册](https://fairino-doc-en.readthedocs.io/latest/CobotsManual/robot_brief_introduction.html)给出移动轴 ZYX 姿态顺序；脚本以等价的固定轴 XYZ 计算 `Rz(rz) @ Ry(ry) @ Rx(rx)`，把 SDK 的毫米和度转换成米和弧度，同时保存六元组原值。[法兰位姿接口](https://fairino-doc-en.readthedocs.io/latest/SDKManual/PythonRobotStatusInquiry.html)为 `GetActualToolFlangePose(0)`。
- [Kalibr 板生成器](https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_create_target_pdf)使用 AprilTag 36h11、2 码元黑边和 180° 旋转的码图；[Kalibr 坐标源码](https://github.com/ethz-asl/kalibr/blob/master/aslam_cv/aslam_cameras_april/src/GridCalibrationTargetAprilgrid.cpp)定义板原点和角点顺序。脚本已按收到的实体照片验证 ID 0、6 和 OpenCV 的角点 0 方向。
- 求解精度仍需真机核对：确认板的实体尺寸、法兰位姿参考系、相机是否牢固，以及留出姿态的板位置/角度一致性。最终机械臂任务精度还受机器人重复定位和工具安装影响。
