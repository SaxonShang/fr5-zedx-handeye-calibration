# FR5 + ZED X 眼在手上标定

这里的脚本使用 FAIRINO Python SDK 读取 FR5 **法兰**位姿，用 ZED SDK 保存 **ZED X 左目已校正图像**和实际内参。标定板固定在工作空间，机器人携相机移动。求解先比较 OpenCV Park–Martin 和 Daniilidis，再以 Park 为初值进行全角点像素重投影优化；结果由留出姿态的重投影误差选择。

## 目录

- `handeye.py`：采集、检测、求解。
- `sim/`：仿真和核对脚本（`synthetic.py`、`fr5_check.py`），不连接相机，也不控制机械臂。
- `tests/`：单元测试；临时文件放在系统临时目录，跑完即删。
- `calibration-data/`：**不进 git** 的本地数据。`reference/` 放数据手册、标定板照片和检测覆盖图，`sim_runs/` 放仿真输出，`datasets/` 放下载的公开数据集；采集的 session 也放在这里。
- `.venv/`：本地 Python 环境，不进 git。

Windows 上建立环境（仓库在中文路径下，OpenCV 不能直接读写这类路径，脚本已改用 `read_image`/`write_image`）：

```powershell
python -m venv .venv
```

```powershell
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

## 开始前

1. 在能使用 ZED SDK 的 Python 环境中安装 `requirements.txt`（OpenCV 限定 4.x，因为 [5.0.0.93 的 Python 包缺失 `calibrateHandEye`](https://github.com/opencv/opencv/issues/29565)）。另外按厂商说明安装 [ZED SDK Python API](https://www.stereolabs.com/docs/app-development/python/install) 和 [FAIRINO Python SDK](https://github.com/FAIR-INNOVATION/fairino-python-sdk)。确认 Python 版本受本机 ZED SDK 支持。
2. [target.example.yaml](target.example.yaml) 根据照片文字填写了 6×6、tagSize `0.055 m`、tagSpacing `0.3`（间隔 `0.0165 m`）。这是**名义尺寸**；收到实际 YAML 后改用它，并用卡尺测量实体板：tag 0 左边缘到 tag 5 右边缘名义为 **412.5 mm**（6×55 + 5×16.5）。打印比例偏 1%，在 0.5 m 距离下手眼平移约偏 5 mm。`tagSpacing` 是间隔与边长之比，不是米。
3. ZED X 是 GMSL2 相机，直接采集需 Jetson + ZED Link/ZED Box；Windows 端可用 [ZED SDK 流接收](https://docs.stereolabs.com/docs/products/cameras/zedx/development-on-pc)，但 Jetson 端需先运行 [官方发送程序](https://github.com/stereolabs/zed-sdk/blob/master/camera%20streaming/single_sender/python/streaming_sender.py)。发送程序在 `init = sl.InitParameters()` 之后要加上 `init.camera_image_flip = sl.FLIP_MODE.OFF` 和 `init.camera_disable_self_calib = True`，与本脚本一致。`--stream-ip` 接收流；不填则打开本机直连相机。
4. 相机为 **ZED X 2.2 mm**（视场 110°×80°，名义 fx ≈ 2.2 mm / 3 µm ≈ 733 px，基线 120 mm，239 g）。用背面 4×M4（螺纹深度 ≤5 mm）或底部 2×M3 加 1/4"-20（≤6.4 mm）固定到法兰支架，不要只靠一颗 1/4" 螺钉：机械臂加减速可能让相机转动，标定随即失效。相机加支架远低于 FR5 的 5 kg 负载。
5. 将标定板牢固固定。采集时手动移动机器人，脚本不会发运动指令。2.2 mm 是广角镜头，相机到板保持 **0.35–0.7 m**：0.4 m 时板约占图像高度 63%，1.0 m 时每个码元只剩约 4 px，检测和姿态精度明显下降。FR5 工作半径 922 mm，板要放在机器人能在该距离环绕观察的位置。建议采集 20–30 个姿态，改变绕至少两个不同轴的转角（各 20–40°），并让标定板覆盖图像不同区域。每次按 Enter 前等待机械臂稳定；脚本会比较抓图前后的法兰位姿，运动超限即丢弃该帧。

## 使用

先用标定板照片确认标签编号和角点方向：

```powershell
python handeye.py detect --image calibration-data/reference/board_photo.jpg --target target.example.yaml --overlay calibration-data/reference/detected.png
```

Kalibr 原版 6×6 板的 tag 0 在左下，5 在右下，30 在左上，35 在右上；每个 tag 的蓝色 `0` 应在该 tag 的**物理右下角**。先前的局部照片检测出 tag 0、6；新提供的整板照片检测出 0–35 全部 36 个标签，角点布局与当前代码一致。诊断照片和覆盖图放在 `calibration-data/reference/`，不随源码提交。若检测编号或角点方向不同，先检查板的制作方式，不能直接套用当前几何映射。

在 Jetson 直连相机时采集：

```powershell
python handeye.py capture --robot-ip 192.168.58.2 --target target.example.yaml --output calibration-data/session_001
```

在接收 ZED SDK 流的主机上采集：

```powershell
python handeye.py capture --robot-ip 192.168.58.2 --stream-ip 192.168.1.10 --target target.example.yaml --output calibration-data/session_001
```

`192.168.58.2` 只是法奥文档中的示例地址，运行时替换为控制器实际 IP；流地址替换为 Jetson 的 IP。`--output` 必须是尚不存在的目录。程序保存 `session.json`、`target.yaml` 和 `images/*.png`，每个 session 保存当次左目内参。分辨率用默认 `HD1200`：`HD1080` 是裁切模式（纵向视场变小），`SVGA` 是合并模式。`VIEW.LEFT` 是已校正图，脚本只配合 `calibration_parameters.left_cam` 使用，不依赖深度图或名义 2.2 mm 焦距。

脚本以 `FLIP_MODE.OFF` 打开相机并关闭 SDK 自标定，两项都记入 `session.json` 和 `result.json`。SDK 默认 `FLIP_MODE.AUTO`：打开时 IMU 判断相机倒置就把图像转 180°，装在机械臂上时这取决于开机姿态。**使用标定结果的程序必须同样设置这两项**，否则左目坐标系可能转 180° 或有细微偏差。旧 session 缺少这两项时，`solve` 会给出警告。

求解：

```powershell
python handeye.py solve --session calibration-data/session_001
```

如果之后测得正确 YAML，求解时可指定 `--target measured_target.yaml`。`result.json` 中 `flange_T_left_camera` 是将左目光学坐标变换到法兰坐标的 4×4 矩阵，平移单位米；`left_camera_T_flange` 是逆矩阵。`base_T_board` 只用于固定板一致性检查。报告同时列出每个算法的训练和留出姿态误差、拒绝样本、PnP 误差和被丢弃的 tag（`dropped_tag_ids`）。至少需要 12 个可用姿态，建议更多。

## 仿真验证

单元测试（`-B` 不生成 `__pycache__`）：

```powershell
.venv\Scripts\python.exe -B -m unittest discover -s tests -v
```

**合成数据端到端**：`sim/synthetic.py` 按实际参数渲染 Kalibr 板图像（412.5 mm 板、ZED X 2.2 mm fx=733、1920×1200、FR5 工作半径内的姿态），生成与 `capture` 格式相同的 session 和 `truth.json`，调用 `solve` 后与真值比较。`suite` 同时跑正常情况和注入的故障，输出到 `calibration-data/sim_runs/`，图像用完即删（`--keep-images` 保留）：

```powershell
.venv\Scripts\python.exe -B sim\synthetic.py suite
```

2026-09-23 的结果（seed 0，25 个姿态，7/7 符合预期）：

| 场景 | 选中结果的误差 | 结论 |
|---|---|---|
| clean | 0.01 mm / 0.001° | 流程本身没有偏差 |
| robot-noise（0.2 mm / 0.02°） | 0.20 mm / 0.010° | 真机误差主要来自机械臂位姿 |
| pose-outliers（3 个位姿偏 3 mm / 0.5°） | 0.59 mm / 0.10° | 能容忍但有影响，每帧都要等机械臂停稳 |
| board-scale（板大 1%） | 4.3 mm | **无法自检**，必须实测板尺寸 |
| flip180（图像被翻转） | 180° | **无法自检**，必须 `FLIP_MODE.OFF` |
| euler-order（欧拉角顺序错） | 留出误差约 300 px | 能明显发现 |
| single-axis（只绕一个轴） | 被拒绝 | 旋转多样性检查生效 |

单个 session 也可以分步生成、求解和比较：

```powershell
.venv\Scripts\python.exe -B sim\synthetic.py make --output calibration-data\sim_runs\demo --robot-noise-mm 0.2 --robot-noise-deg 0.02
```

```powershell
.venv\Scripts\python.exe -B handeye.py solve --session calibration-data\sim_runs\demo
```

```powershell
.venv\Scripts\python.exe -B sim\synthetic.py compare --session calibration-data\sim_runs\demo
```

**FR5 位姿约定**：`sim/fr5_check.py` 只读，不发运动指令。连接 [FAIRINO SimMachine](https://fairino-doc-en.readthedocs.io/latest/download.html) 或真机，在网页界面或示教器上手动点动到 6 个以上姿态（每个关节都要变化，J4–J6 至少 30°），每个姿态按一次 Enter。脚本读取关节角和 `GetActualToolFlangePose(0)`，用手册的 FR5 DH 参数（已与官方 URDF 核对，差 ≤0.1 mm）计算正运动学，拟合常量基座和法兰偏移后比较三种姿态约定：

```powershell
.venv\Scripts\python.exe -B sim\fr5_check.py --robot-ip <SimMachine IP> --output calibration-data\fr5_check.json
```

`PASS` 表示 `handeye.py` 的姿态约定和单位正确。SimMachine 上残差应小于 0.5 mm；真机可能因出厂运动学标定有几毫米残差，而错误约定会差几十度。

## 坐标和可靠性核对

- [FAIRINO 手册](https://fairino-doc-en.readthedocs.io/latest/CobotsManual/robot_brief_introduction.html)给出移动轴 ZYX 姿态顺序；脚本以等价的固定轴 XYZ 计算 `Rz(rz) @ Ry(ry) @ Rx(rx)`，把 SDK 的毫米和度转换成米和弧度，同时保存六元组原值。[法兰位姿接口](https://fairino-doc-en.readthedocs.io/latest/SDKManual/PythonRobotStatusInquiry.html)为 `GetActualToolFlangePose(0)`。
- [Kalibr 板生成器](https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_create_target_pdf)使用 AprilTag 36h11、2 码元黑边和 180° 旋转的码图；[Kalibr 坐标源码](https://github.com/ethz-asl/kalibr/blob/master/aslam_cv/aslam_cameras_april/src/GridCalibrationTargetAprilgrid.cpp)定义板原点和角点顺序。脚本已按收到的整板照片验证 ID 0–35 和 OpenCV 的角点 0 方向。
- 角点检测：Kalibr 板间隙处的小方块与每个 tag 角相接，角点是 X 形交点。OpenCV `ArucoDetector` 在这里偏 1–2 px（偶尔十几像素）。`detect_board` 按间隙大小设定窗口再做一次 `cornerSubPix`（合成图平均误差 0.03 px），`solve` 再按整板 PnP 残差丢弃锁错特征的 tag。真机图像的 `pnp_rmse_px` 应明显低于 1 px；如果普遍接近 1–2 px，先检查检测。
- 结果合理性：ZED X 基线 120 mm，相机外壳横向中心位于左目光学坐标 +x 约 60 mm 处。用支架 CAD 估算 `flange_T_left_camera` 的平移，与结果相差超过 5–10 mm 时先排查板尺寸、相机固定和位姿约定。
- 求解精度仍需真机核对：确认板的实体尺寸、法兰位姿参考系、相机是否牢固，以及留出姿态的板位置/角度一致性。FR5 重复定位精度为 ±0.02 mm，但手眼求解依赖控制器报告的绝对位姿，留出姿态的板位置误差也包含机器人绝对精度的影响。最终机械臂任务精度还受工具安装影响。
