# FR5 + ZED X 眼在手上标定

用 Kalibr Aprilgrid 标定板，求 **ZED X 左目相机相对 FR5 法兰的位姿** `flange_T_left_camera`。

- **输入**：标定板图像、每张图对应的法兰位姿（最好附关节角）、相机内参。
- **输出**：手眼结果，以及说明结果可信程度的质量报告。

图像和位姿由你们自己的系统采集，本仓库只负责标定，不依赖 ZED 或 FAIRINO 的 SDK。维护和修改代码前，先读 [DEVELOPMENT.md](DEVELOPMENT.md)。

## 快速上手

**0. 安装环境**（一次即可，详见 [1.4](#14-软件环境)）

```bash
python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt
```

**1. 新建 session 文件夹**。从 `templates/session/` 复制出 `camera.yaml`、`target.yaml`、`poses.csv`、`touch_points.csv`，并建好空的 `images/`：

```bash
.venv/bin/python -m handeye init --session calibration-data/sessions/s001
```

**2. 填写配置**（写法见 [2.1](#21-session-文件夹与配置文件)）

| 文件 | 填什么 | 最快的办法 |
|---|---|---|
| `camera.yaml` | 校正后左目内参 | 在 ZED Box 上运行 `python3 tools/zed_camera_yaml.py --output camera.yaml`，再拷过来覆盖 |
| `target.yaml` | 标定板尺寸 | 使用已确认准确的 6×6、55 mm、间隙 16.5 mm |
| `images/` | 每个机位一张图 | 从 Box 拷过来 |
| `poses.csv` | 每张图一行：法兰位姿 + 6 个关节角 | 照抄 FR5 WebApp 的显示值 |

**3. 核对位姿表**（可选，需要关节角）

```bash
.venv/bin/python -m handeye check-poses --poses calibration-data/sessions/s001/poses.csv
```

**4. 标定**。`--expected-translation-mm` 是事先粗估的"左目光心在法兰坐标系中的位置"（mm），只用来核对结果，估法见 [2.1](#21-session-文件夹与配置文件) 末尾。下面的 `60 -75 45` 只是示例，要换成自己的估计值：

```bash
.venv/bin/python -m handeye solve --session calibration-data/sessions/s001 --expected-translation-mm 60 -75 45
```

屏幕输出和 `result.json` 的读法见 [第 4 节](#4-结果评估与诊断)。

**5. 复核**（推荐）。换一组不同的姿态再建一个 session、再标定一次，比较两次的结果：

```bash
.venv/bin/python -m handeye compare --results calibration-data/sessions/s001/result.json calibration-data/sessions/s002/result.json
```

第一次上真机，请按 [第 5 节](#5-上真机试标清单) 的试标清单逐项核对。

**没有硬件也能先跑一遍**：用仿真生成一个与真实数据格式完全相同的 session，再对它标定：

```bash
.venv/bin/python -B sim/synthetic.py make --output calibration-data/sim_runs/demo --robot-noise-mm 0.2 --robot-noise-deg 0.02
```

```bash
.venv/bin/python -m handeye solve --session calibration-data/sim_runs/demo
```

Windows 上把 `.venv/bin/python` 换成 `.venv\Scripts\python.exe`。

## 目录结构

```text
calibration/
├─ README.md
├─ DEVELOPMENT.md            维护与交接说明（约定、设计原因、已知问题）
├─ requirements.txt
├─ handeye/                  标定算法，用 python -m handeye 运行
│  ├─ __main__.py            命令行：init / detect / check-poses / solve / compare / validate
│  ├─ dataset.py             读取 session（camera.yaml、poses.csv、images/）
│  ├─ board.py               Kalibr Aprilgrid 几何与角点检测
│  ├─ observations.py        单张图的 PnP 与锁错 tag 剔除
│  ├─ solver.py              手眼初值、重投影优化、交叉验证选方法、质量门限
│  ├─ diagnostics.py         覆盖度、jackknife 不确定度、内参/畸变/板尺度诊断
│  ├─ kinematics.py          FR5 运动学、欧拉角约定、位姿-关节角一致性、TCP/工件坐标系检查
│  ├─ validation.py          尖端点独立验证、多次结果比较
│  └─ geometry.py            变换、FR5 位姿约定、图像读写
├─ templates/session/        新 session 的模板，init 会复制它
├─ tools/zed_camera_yaml.py  在 ZED Box 上导出 camera.yaml
├─ sim/synthetic.py          仿真数据生成与验证套件
├─ tests/                    单元测试
└─ calibration-data/         本地数据，不进 git
   ├─ sessions/              真实采集的 session
   ├─ sim_runs/              仿真输出
   ├─ reference/             数据手册、标定板照片、硬件审查报告
   └─ archive/               早先基于 SDK 的自动采集代码，仅供参考
```

## 1. 硬件与环境

### 1.1 硬件清单

| 设备 | 型号与要点 |
|---|---|
| 机械臂 | FAIRINO FR5：负载 5 kg，工作半径 922 mm，重复定位精度 ±0.02 mm |
| 相机 | ZED X，2.2 mm 镜头：1920×1200 全局快门，视场 110°×80°，名义 fx ≈ 733 px，基线 120 mm，239 g |
| 计算单元 | ZED Box Mini（Orin NX 16GB）：600 g，12 V（10.5–13.5 V），最大 60 W，Jetson 上已装好 ZED SDK |
| 标定板 | Kalibr Aprilgrid 6×6，tag 边长 55 mm，间隙 16.5 mm，tag 阵列外缘 412.5 mm |
| 电脑 | 笔记本（Ubuntu 22.04 或 Windows），只运行本仓库 |

### 1.2 连接方式

```text
FR5 末端法兰
 └─ 刚性支架 ─┬─ ZED X 2.2 mm
              └─ ZED Box Mini ──HDMI── 显示器（查看、保存图像）
                  GMSL2 短线，与相机一起随末端运动；12 V 电源线沿机械臂走线

笔记本 ──网线── FR5 控制器：用 WebApp 移动机械臂，读取位姿和关节角
图像从 Box 拷到笔记本的 session/images/ 下
```

每个机位：用 WebApp 把机械臂移到位 → 等它完全停稳 → 在 Box 上保存一张图 → 在 WebApp 上抄下法兰位姿和 6 个关节角。

### 1.3 安装与采集要点

下面几项决定精度，其中标 ⚠️ 的算法无法事后发现或纠正。

- ⚠️ **图像不能被翻转**：ZED 以 `FLIP_MODE.OFF` 打开。SDK 默认的 `AUTO` 在相机倒置时会把图像转 180°，标定结果也随之转 180°，任何检查都发现不了。
- **自标定**：开或关都可以，但采集和实际使用必须一致。
- **预热**：Box 紧挨相机、会发热，开机后等 20–30 分钟再采集；实际使用时也保持同样的热状态。
- ⚠️ **支架刚性**：相机尽量靠近法兰、直接固定，Box 的 600 g 不要经过相机的安装件传递。支架随姿态变化的变形无法被标定消除。
- **线缆**：电源线在支架上固定好，给 J4–J6 留足旋转余量，不能拉扯末端。
- **负载**：末端总重约 1.1–1.3 kg，远低于 5 kg，但要在控制器里设置负载质量和质心。
- **标定板**：按已确认准确的理论尺寸使用，牢固固定并保持平整（见 [2.1](#21-session-文件夹与配置文件)）。

### 1.4 软件环境

**电脑**：需要 Python ≥ 3.9。Ubuntu 22.04 自带的 3.10 即可；不需要 ZED 或 FAIRINO 的 SDK。

```bash
sudo apt install python3-venv
```

```bash
python3 -m venv .venv
```

```bash
.venv/bin/python -m pip install -r requirements.txt
```

Windows 用 `python -m venv .venv` 和 `.venv\Scripts\python.exe -m pip install -r requirements.txt`。依赖为 numpy、OpenCV contrib 4.x、SciPy、PyYAML。OpenCV 限定 4.x，因为 [5.0.0.93 的 Python 包缺失 `calibrateHandEye`](https://github.com/opencv/opencv/issues/29565)。仓库放在中文路径下也能正常读写图像。

此前测试过的依赖组合（Windows；本轮测试数量见开发说明）：

| Python | opencv-contrib-python | numpy | scipy | PyYAML |
|---|---|---|---|---|
| 3.10.21 | 4.14.0 | 2.2.6 | 1.15.3 | 6.0.3 |
| 3.14.7 | 4.14.0 | 2.5.3 | 1.18.1 | 6.0.3 |

第一行是 pip 在 Python 3.10 下自动选出的版本，Ubuntu 22.04 上应该相同。到 Ubuntu 上先跑一遍测试确认：

```bash
.venv/bin/python -B -m unittest discover -s tests
```

**ZED Box**：需要 ZED SDK、`pyzed` 和 PyYAML（`python3 -m pip install PyYAML`）；只需把 `tools/zed_camera_yaml.py` 拷过去运行，用来导出内参（尚未在真机测试；须与保存图像的采集程序使用相同配置和校正后左目内参，不能直接与 Explorer 的 raw 内参混用）。

## 2. 标定算法

### 2.1 Session 文件夹与配置文件

```text
calibration-data/sessions/s001/
├─ camera.yaml       校正后左目内参
├─ target.yaml       标定板定义
├─ poses.csv         每张图一行的位姿表
├─ images/           图像（PNG 等无损格式）
└─ touch_points.csv  可选，独立验证用
```

**camera.yaml**：ZED **校正后左目**（`VIEW.LEFT`）的内参，分辨率必须与图像一致。

```yaml
image_width: 1920
image_height: 1200
fx: 733.0          # 以下四个值换成实际读出的值
fy: 733.0
cx: 959.5
cy: 599.5
distortion: [0.0, 0.0, 0.0, 0.0, 0.0]   # 校正图为 0
```

数值来自 SDK 的 `calibration_parameters.left_cam`。`tools/zed_camera_yaml.py --output camera.yaml` 直接写入文件，避免 SDK 的终端日志混入 YAML，同时记录实际分辨率、帧率、序列号和 SDK 版本。**不要用 `SN*.conf` 或 Explorer 的 raw 内参代替**，它们对应未校正图像。

导出工具会单独打开相机，默认关闭启动自标定。图像采集与实际使用必须采用同一相机、SDK、分辨率、翻转和自标定设置；如果开启启动自标定，必须从**保存图像的同一个相机实例**导出 K，不能拿另一次启动生成的 YAML 为已有图像作保证。仅有元数据相同也不能证明 K 与图像来自同次标定。

显式 `image_flip: ON/AUTO`、非 `LEFT` 或未校正图像声明会被拒绝。旧文件缺少这些字段仍可读取，缺失不代表相机设置已核验。

**target.yaml**：按你确认的准确理论值固定为 6×6、`tagSize: 0.055`、`tagSpacing: 0.3`（间隙与边长之比，不是米），间隙 16.5 mm，tag 阵列外缘 **412.5 mm**。主求解不估计或修改板尺寸；尺度诊断只检查图像、内参与机械臂数据是否和这组准确尺寸一致。

**poses.csv**：以 `#` 开头的行是注释。

```text
image,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg
images/0001.png,512.340,-35.120,610.450,-176.320,8.140,31.500,-12.300,-85.400,92.100,-96.800,-88.500,40.200
```

- `x_mm … rz_deg` 是**法兰**（工具坐标系 0）在**基坐标系**（工件/用户坐标系 0）下的位姿，照抄 WebApp 的显示值。读数前确认 WebApp 当前激活的工具和工件坐标系都是 0。
- `j1_deg … j6_deg` 是同一时刻的关节角。它们是可选的，但强烈建议填写，因为可以用来找出抄错的行、核对欧拉角约定。要么六列都填，要么连同表头一起删掉。
- 报错信息会指明是哪一行出了问题。

**touch_points.csv**：可选，见 [4.5](#45-重复性与独立验证)。

**支架估计**（`solve --expected-translation-mm X Y Z`）：手眼结果 `flange_T_left_camera` 的平移，就是**左目光心在法兰坐标系中的坐标**。事先用支架 CAD 或尺子粗估这三个数；算出的平移与估计值相差超过 `--expected-tolerance-mm`（默认 10 mm）时，结果判为 `rejected`。它不参与求解，只用来发现整体性错误（例如位姿读成了 TCP、坐标轴或符号弄错）；不给出时只发警告。

- **法兰坐标系**：原点在法兰端面中心，+Z 通常垂直于端面朝外。三个轴的方向在 WebApp 里确认：选工具坐标系 0，用"工具坐标"模式分别点动 +X、+Y、+Z，看法兰往哪边走。
- **左目光心**：从相机背后、顺着镜头朝向看，是左边那个镜头的中心，离外壳横向中心 60 mm（基线 120 mm）；深度取镜头前表面往里几毫米即可。
- **三个数**：从法兰中心出发，沿法兰的 X、Y、Z 轴分别量到左目光心的距离，带正负号。只看光心位置，与相机朝向无关。误差在 ±5 mm 内就够用；估得更粗时，可以把 `--expected-tolerance-mm` 调大。
- **例子**：左目光心在法兰 +X 方向 60 mm、−Y 方向 75 mm，并且在法兰端面外侧 45 mm，就填 `--expected-translation-mm 60 -75 45`。
- **只因这一项被拒绝时**：把屏幕上打印的 `flange_T_left_camera translation [...]` 与估计值逐轴对比。如果只有某一轴的正负号相反，多半是估计时把轴的方向弄反了。

### 2.2 采集姿态建议

- 20–30 个姿态；相机到板的距离 0.35–0.7 m（2.2 mm 是广角镜头，1.0 m 时每个码元只剩约 4 px）。
- 绕至少两个不同的轴旋转，幅度 20–40°，包括绕相机光轴的滚转；板相对视线倾斜 20–40°。
- 让标定板出现在图像的各个区域，包括边缘和角落。
- 避开奇异位形：J5 接近 0° 或 180°、J3 接近 0°、腕部中心接近 J1 轴线。
- 每个姿态都要等机械臂完全停稳再拍照和读数；不要在拖动模式下读数。

### 2.3 数学模型与处理流程

每张图 i 满足：

```text
base_T_flange_i · X · camera_T_board_i = Y
X = flange_T_left_camera（要求的手眼结果）   Y = base_T_board（标定板在基坐标系中的位姿）
```

1. **检测角点**：用 OpenCV 检测 AprilTag 36h11（2 码元黑边）。Kalibr 板间隙处的小方块与每个 tag 角相接，角点是 X 形交点，OpenCV 在这里偏 1–2 px，所以再按间隙大小做一次 `cornerSubPix`（合成图平均误差 0.03 px）。
2. **单帧 PnP**：得到 `camera_T_board`，按整板残差丢弃锁错特征的 tag。剩下的完整 tag 必须至少 4 个，并跨越两行两列。
3. **候选解**：
   - Park、Daniilidis：相对运动形式 AX = XB 的闭式解；
   - 以 Park 为初值，对 X 和 Y 联合最小化全部角点的重投影误差（Huber 损失，机械臂位姿视为准确）。
4. **选方法**：每 5 个可用视图取 1 个作为**测试视图**，只用于报告和判定；在其余训练视图上做 4 折交叉验证，选交叉验证误差最小的方法，再用全部训练视图拟合。候选资格只看训练集和 CV；选定方法的测试结果无效时直接拒绝，不按测试成绩换方法。
5. **质量门限与诊断**：见 [第 4 节](#4-结果评估与诊断)。

### 2.4 命令一览

| 命令 | 作用 |
|---|---|
| `init --session DIR [--camera FILE] [--intrinsics FX FY CX CY] [--size W H] [--target FILE]` | 按模板新建 session；可直接写入内参 |
| `detect --image IMG [--overlay OUT]` | 检查一张图的 tag 编号和角点方向（每个 tag 的蓝色 `0` 应在该 tag 的物理右下角） |
| `check-poses --poses CSV` | 用 FR5 运动学核对位姿表：欧拉角约定是否为 `Rz·Ry·Rx`、有没有与关节角不一致的行、读数时是否激活了 TCP 或工件坐标系 |
| `solve --session DIR [选项]` | 标定，写出 `result.json` |
| `compare --results A.json B.json [...]` | 比较同一安装状态下多次独立标定的结果 |
| `validate --result JSON --points CSV [--session NEW_DIR]` | 用尖端点做独立验证 |

`solve` 的常用选项：
- 核对与判定：`--expected-translation-mm X Y Z`、`--expected-tolerance-mm`（默认 10），以及 [4.3](#43-质量门限) 里的各项门限。
- 检测与求解：`--target`、`--min-tags`（4）、`--max-pnp-rmse`（2 px）、`--no-refine`。
- 诊断：`--bootstrap`（jackknife 子集数，默认 30，0 为关闭）、`--max-scale-error`（0.003）、`--max-focal-error`（0.003）、`--max-principal-point-px`（3）、`--max-distortion-px`（1）。
- `--allow-tcp-offset`：确实要标定相机相对某个 TCP 的位姿时使用，这时位姿读成 TCP 只警告、不拒绝。

运行 `python -m handeye solve --help` 查看全部选项。

## 3. 仿真验证

`sim/synthetic.py` 按实际参数渲染 Kalibr 板图像：板 412.5 mm，ZED X 2.2 mm（fx=733），1920×1200。姿态必须有 FR5 名义逆解、在 URDF 关节限位内，并远离奇异位形。输出格式与真实 session 完全相同，另附真值 `truth.json` 和尖端点 `touch_points.csv`。

```bash
.venv/bin/python -B -m unittest discover -s tests -v
```

```bash
.venv/bin/python -B sim/synthetic.py suite
```

```bash
.venv/bin/python -B sim/synthetic.py compare --session calibration-data/sim_runs/demo
```

- `make` 的注入选项：`--robot-noise-mm/--robot-noise-deg`、`--board-scale`、`--k-error`、`--residual-k1`、`--flip180`、`--euler-order xyz`、`--tcp-offset-mm`、`--user-frame`、`--pose-outliers`、`--pose-typo`、`--joint-offset-deg`、`--jpeg-quality`、`--single-axis`。
- `suite` 的"检查"场景事先写明预期，有一项不符就返回退出码 1；"量化"场景只报告数值。输出放在 `calibration-data/sim_runs/`，图像用完即删。

2026-09-24 的结果（seed 0，25 个姿态）。"正确"指误差 ≤ 2 mm 且 ≤ 0.2°。警告标记：S 板尺度、K 内参、D 残余畸变、R 位姿行、T 位姿读成 TCP、U 位姿在工件坐标系下。

| 场景 | 类型 | 判定 | 与真值的误差 | jackknife sd | 正确 | 警告 |
|---|---|---|---|---|---|---|
| clean | 检查 | accepted | 0.003 mm / 0.000° | 0.004 mm | 是 | — |
| 位姿噪声 0.2 mm / 0.02° | 检查 | accepted | 0.11 mm / 0.022° | 0.16 mm | 是 | — |
| 3 个位姿偏 3 mm / 0.5° | 检查 | accepted | 0.11 mm / 0.043° | 0.75 mm | 是 | R |
| 一行 x 多抄 10 mm | 检查 | accepted | 0.55 mm / 0.090° | 0.85 mm | 是 | R（指出第 4 行） |
| 板大 1% | 检查 | accepted | 4.6 mm / 0.040° | 0.53 mm | 否 | S |
| 内参：焦距 +0.5%，主点 3 px | 检查 | accepted | 2.3 mm / 0.28° | 0.24 mm | 否 | K |
| 残余畸变 k1 = −0.0008（角落约 2 px） | 检查 | accepted | 0.07 mm / 0.001° | 0.007 mm | 是 | D |
| 位姿读成 120 mm 的 TCP | 检查 | **rejected** | 120 mm | 0.004 mm | 否 | T |
| 位姿读在工件坐标系下 | 检查 | accepted | 0.004 mm / 0.001° | 0.005 mm | 是 | U |
| 图像转 180° | 检查 | accepted | 180° | 0.004 mm | **否** | **无** |
| 欧拉角约定错误 | 检查 | rejected | 702 mm / 179° | — | 否 | S T U |
| 只绕一个轴旋转 | 检查 | insufficient_data | — | — | — | — |
| 位姿噪声 0.5 mm / 0.05° | 量化 | accepted | 0.28 mm / 0.053° | 0.40 mm | 是 | — |
| 关节零位系统误差 0.02° | 量化 | accepted | 0.12 mm / 0.061° | 0.14 mm | 是 | — |
| JPEG q70（有损视频的替代） | 量化 | accepted | 0.004 mm / 0.000° | 0.005 mm | 是 | — |
| 测试视图中 2 个位姿有误 | 检查 | rejected | 0.003 mm / 0.000° | 0.005 mm | 是 | R |
| 噪声 + 板 1% + 内参误差 | 量化 | accepted | 2.4 mm / 0.30° | 0.79 mm | 否 | S K |

从这张表可以看出：
- 检查场景 13/13 符合预期（含测试视图异常拒绝检查）。
- 结果有错时，除了"图像转 180°"，其余都会被拒绝或给出警告；正常数据（包括带位姿噪声的）不产生诊断警告。
- jackknife sd 与真实误差在同一量级，但它**反映不了**板尺寸、内参这类所有视图共有的误差。

仿真的局限：渲染与检测共用同一板定义；内参为名义值；FR5 只用名义运动学，未检查碰撞；有损视频只用 JPEG 近似。因此仿真只验证所覆盖场景的实现和检查逻辑，**不能证明整体正确性或真机精度**。

## 4. 结果评估与诊断

### 4.1 看懂输出

下面是仿真数据（位姿噪声 0.2 mm / 0.02°，20 个姿态，给出了支架 CAD 平移）的实际输出：

```text
Views: 20 in poses.csv, 20 usable, 0 rejected; train 16, test 4          ← 可用 / 被拒绝的视图数
  Park                   CV 0.753 px | test 0.566 px, board 0.45 mm / 0.041 deg
  Daniilidis             CV 0.758 px | test 0.559 px, board 0.44 mm / 0.040 deg
  Park+pixel_refinement  CV 0.851 px | test 0.631 px, board 0.63 mm / 0.079 deg
Chosen Park: flange_T_left_camera translation [-60.00, 75.05, 44.93] mm      ← 按 CV 选出
  jackknife sd: translation 0.181 mm [0.0995, 0.0445, 0.1451], rotation 0.0199 deg (random errors only)
  board scale (robot-metric fit): 0.9996x target.yaml
  intrinsics from the images vs camera.yaml: focal +0.00% / +0.00%, principal point (-0.0, +0.0) px,
    residual distortion 0.07 px at the image corner
STATUS: ACCEPTED
WARNING: the board covered only 40% of the image over all views; move it towards the image edges and corners too
```

- **CV**：交叉验证的角点误差，用来选方法。
- **test**：测试视图上的角点误差，以及把每个视图算出的板位姿放在一起时的离散程度（板是固定的，理想情况为 0）。
- **jackknife sd**：结果的随机不确定度。
- **board scale**：相对于固定准确板尺寸的模型一致性诊断，接近 1 为正常；未收敛时只显示状态和警告。
- **intrinsics from the images**：只用图像重新估计的内参和残余畸变与 `camera.yaml` 的差，接近 0 为正常。

### 4.2 状态与退出码

| 状态 | 含义 | 退出码 |
|---|---|---|
| `accepted` | 通过全部质量门限 | 0 |
| `rejected` | 某项门限超限，或位姿读成了 TCP；原因写在 `reasons` 里 | 2 |
| `insufficient_data` | 可用视图少于 12 个，或旋转缺少两个不同的轴 | 2 |

程序出错时退出码为 1。三种状态都会写出 `result.json`。`accepted` 只表示内部一致性检查通过，**不是实测精度**。

### 4.3 质量门限

门限是本项目的默认值，应按任务要求和到货实测调整，不是厂家指标。

| 检查项 | 选项 | 默认 |
|---|---|---|
| 测试视图角点 RMSE | `--max-test-px` | 2.0 px |
| 最差单个测试视图的 RMSE | `--max-view-px` | 4.0 px |
| 固定板位置一致性（测试视图） | `--max-board-mm` | 3.0 mm |
| 固定板角度一致性（测试视图） | `--max-board-deg` | 0.5° |
| 与支架 CAD 的平移偏差 | `--expected-translation-mm`、`--expected-tolerance-mm` | 容差 10 mm；不给出时只警告 |
| 位姿读成 TCP（需要关节角） | `--allow-tcp-offset` 明确允许 TCP 输出 | 法兰偏移 > 5 mm 或 > 0.5° 即拒绝 |

### 4.4 诊断与处理

下表中的诊断只给出警告，不会改变标定结果。

| 警告 | 含义 | 处理 |
|---|---|---|
| `board scale consistency fit is …x target.yaml` | 准确板尺寸与当前模型的尺度一致性偏差超过 0.3% | 检查内参、机械臂位姿、图像与位姿配对；保持准确板尺寸不变。诊断不收敛时显示 inconclusive/failed，不报告可信尺度 |
| `focal length from the images …` 或 `principal point from the images …` | 仅用图像估计的焦距偏离超过 0.3%，或主点偏离超过 3 px（且超过 3 倍标准差） | 核对 `camera.yaml` 是否为校正后左目、分辨率是否一致 |
| `residual radial distortion in the images …` | 图像里还有畸变，角落偏移超过 1 px | 确认存的是校正后的左目图，而不是原始图 |
| `poses.csv row N: pose disagrees with its joints` | 这一行的位姿与关节角对不上 | 检查抄写，以及位姿和关节角是否在同一时刻读取 |
| `poses look like they are in a user/work frame` | 读数时激活了工件坐标系 | 手眼结果仍相对法兰；板位姿改为 `pose_reference_T_board`，不再导出 `base_T_board`，不能直接与基坐标系触点比较 |
| `poses.csv has no joint angles` | 没有关节角，无法做以上两项检查 | 补上关节角，或至少给出 `--expected-translation-mm` |
| `… differs from … by … mm` | 不同方法的结果相差较大 | 增加姿态、加大旋转幅度，检查位姿数据 |
| `board covered only …%`、`board tilt only …`、`largest robot rotation …` | 姿态覆盖不足 | 按 [2.2](#22-采集姿态建议) 补拍 |
| 某个视图出现在 `rejected_views` 里 | 该图检测失败或 PnP 误差大 | 看给出的原因：tag 太少、太远、模糊或过曝 |

如果拒绝原因是 `poses look like an active TCP`，说明读数时 WebApp 激活了工具坐标系，结果会是相机相对那个 TCP 的位姿，而不是相对法兰（仿真中差了整整 120 mm）。处理办法：把工具坐标系设为 0 后重新读取位姿。若明确使用 `--allow-tcp-offset`，输出 `pose_moving_T_left_camera` 和 `frames.pose_moving=reported_tcp`，不提供法兰别名，也不能直接做法兰 CAD 核对、重复性比较或基坐标系验收。名义运动学拟合的偏移不作为真实坐标转换。

这些诊断的原理：

- **内参与残余畸变只看图像**：每张图的板位姿各自自由，只用图像重新估计 fx、fy、cx、cy 和径向畸变 k1（相当于一次普通的相机标定），再与 `camera.yaml` 比较。因为完全不用机械臂位姿，所以机械臂误差漏不进来。最初的做法是和机械臂数据联合拟合，结果 0.5 mm 的位姿噪声被当成了 2.7 px 的畸变，因此改成了现在的方式。这项检查看不出板尺寸，并且假设标定板是平的。
- **尺度一致性检查要结合机械臂**：单独放开尺度进行诊断拟合，结果不反馈给主求解。已知板尺寸准确时，偏离 1 说明当前内参、位姿或数据配对等模型存在不一致；仅靠这个值不能确定原因。仿真中的板尺度误差是故障注入，不改变实体板的准确尺寸假设。
- **位姿坐标系**：把 FK(关节角) 与记录的位姿对齐，所需的常量偏移在法兰端就是 TCP，在基座端就是工件坐标系。FR5 的 DH 法兰就是控制器的法兰，所以正常情况下这两个偏移都接近 0。
- **jackknife 不确定度**：对训练视图反复抽取 80% 的子集重新求解，由结果的离散程度估计误差，其中包含机械臂位姿噪声的影响。雅可比协方差只计入像素噪声，仿真中低估约 10 倍，所以不用它。

### 4.5 重复性与独立验证

**重复性**（不需要额外工具，推荐每次都做）：安装不动，换一组不同的姿态再建一个 session、再标定一次，然后比较：

```bash
.venv/bin/python -m handeye compare --results calibration-data/sessions/s001/result.json calibration-data/sessions/s002/result.json
```

仿真示例：两组各 20 个姿态，结果相差 0.148 mm，是 jackknife 随机误差尺度（0.225 mm）的 0.7 倍，判定为一致。
- 差异超过 2 mm / 0.2°，或超过随机误差尺度的 3 倍时，判为 `DIFFERENT`，退出码 2。这说明两次之间有东西变了：支架松动、热状态不同、位姿坐标系不同等。
- 一致也**不能**排除两次共有的误差，比如板尺寸、内参、图像翻转。

**板位姿检查**（需要尖端工具）：用已知 TCP 的尖端工具，测量板角点在机械臂基坐标系中的位置，填写 `touch_points.csv`。至少 3 个不同且不共线的角点，推荐 0 BL、5 BR、30 TL、35 TR 加内部角。程序拒绝重复、非有限和近共线数据；第二方向的 RMS 展布需至少 10 mm，这是项目默认值。

```bash
.venv/bin/python -B -m handeye validate --result calibration-data/sessions/s001/result.json --points calibration-data/sessions/s001/touch_points.csv
```

输出 `scope=board_pose_check`。它比较保存的 **Y（板位姿）** 与实测触点，报告每点误差、RMS、板位姿差和触点拟合残差。**单凭这项通过，不能认定 X（手眼变换）正确。**

**独立手眼链检查**：保持支架和板的位置不变，另外采集一个未用于拟合的 session（例如 `validation01`），使用同一相机配置及准确板定义，仍记录工具 0、工件坐标系 0 的法兰位姿。至少 3 个可用视图，旋转需覆盖两个不同的轴；建议多采几个分散姿态。运行：

```bash
.venv/bin/python -B -m handeye validate --result calibration-data/sessions/s001/result.json --points calibration-data/sessions/s001/touch_points.csv --session calibration-data/sessions/validation01 --output calibration-data/sessions/validation01/validation.json
```

输出 `scope=handeye_chain_check`。每个新视图使用 `base_T_flange · 已有X · camera_T_board` 预测板角点，再与触点比较；不使用保存的 Y，也不重新拟合 X。总 RMS 或最差视图 RMS 超过 `--max-rms-mm`（默认 3 mm）时退出码为 2。

- `validate` 和 `compare` 只接受当前格式且 `accepted` 的结果。旧结果缺少 `frames` 时，重新运行 `solve`。
- `compare` 至少需要两个不同文件、不同来源 session；不能把同一份结果换名字当作重复标定。原图存在时会检查是否复用像素；原图缺失时明确提示独立性仅依赖记录的路径。
- 独立链检查要求原标定图像仍可读取，并比较解码图像哈希以排除原图复制；不要删除原 session 的图像。路径迁移后需更新数据位置并重新 `solve`。
- 尖端 TCP 自身误差、机械臂误差会进入结果；共享的图像翻转、内参或坐标系声明错误仍可能相互抵消。通过这些检查不能代替最终任务测试。

### 4.6 精度预期

当前合成模型与固定随机种子下，输入准确时误差约为 0.003 mm。这验证了这些场景中的数值实现，不能证明算法普遍无偏。实际精度取决于输入：

| 输入误差（仿真，25 个姿态） | 对结果的影响 | 能否发现 |
|---|---|---|
| 角点检测噪声 | < 0.01 mm | — |
| 位姿噪声 0.2 mm / 0.02° | 0.11 mm / 0.02° | jackknife sd 能反映量级 |
| 板尺寸 +1% | 4.6 mm | 能：板尺度诊断 |
| 内参：焦距 +0.5%，主点 3 px | 2.3 mm / 0.28° | 能：只用图像的内参检查 |
| 残余畸变（角落约 2 px） | 0.07 mm | 能：只用图像的畸变检查 |
| 位姿抄错 10 mm | 0.55 mm | 能：指出具体行 |
| 位姿读成 TCP | 等于 TCP 偏移（120 mm） | 能：被拒绝（需要关节角或 CAD 核对） |
| 欧拉角约定错误 | 数百毫米 | 能：被拒绝 |
| 图像转 180° | 180° | **不能**，只能靠 `FLIP_MODE.OFF` 预防 |

实际精度还取决于 FR5 位姿的**绝对精度**、内参、图像质量、安装刚性和数据配对。重复定位精度不能代替绝对精度，目前没有真机数据可支持亚毫米等精度承诺。**应使用独立 session 的 `validate --session` 和任务层面的测试确定可用精度。**

### 4.7 `result.json` 主要字段

| 字段 | 内容 |
|---|---|
| `status`、`reasons`、`warnings` | 判定、拒绝原因、警告 |
| `schema_version`、`frames` | 结果格式版本 2，以及位姿的移动端、参考端和相机坐标系声明；无关节角时无法交叉核对声明 |
| `chosen.pose_moving_T_left_camera`、`chosen.pose_reference_T_board` | 始终存在的变换；含义以 `frames` 为准 |
| `chosen.flange_T_left_camera` | 仅移动端为 flange 时输出。手眼结果：4×4 矩阵，把左目光学坐标（x 右、y 下、z 沿光轴向前）变换到法兰坐标，平移单位米 |
| `chosen.left_camera_T_flange` | 上面的逆矩阵 |
| `chosen.base_T_board` | 仅参考系为 base 时输出；检测到工件坐标系时不提供此别名 |
| `uncertainty` | jackknife 的平移标准差（mm）和旋转标准差（°） |
| `intrinsics_check` | 仅用图像估计的 fx、fy、cx、cy、k1 及其与 `camera.yaml` 的差和标准差 |
| `board_scale_check` | 固定准确板尺寸下的模型一致性诊断，含收敛 status；失败时不输出尺度估计 |
| `pose_joint_consistency` | 逐行位姿与关节角的一致性、可疑行，以及拟合出的 TCP / 工件坐标系偏移 |
| `data_coverage` | 图像覆盖率、距离、倾角、最大转角 |
| `test_views`、`candidates`、`selection` | 每个测试视图的误差，各方法的交叉验证 / 训练 / 测试误差，选择规则 |
| `accepted_views`、`rejected_views` | 每张图的 tag、被丢弃的 tag、PnP 误差或被拒原因 |

## 5. 上真机试标清单

第一次在真机上标定时，按顺序逐项确认。每一项都针对一个仿真无法覆盖的环节。

1. **环境**：在 Ubuntu 笔记本上跑一遍测试（[1.4](#14-软件环境)），全部通过。
2. **硬件**：支架刚性、线缆固定、控制器里设好负载；Box 开机预热 20–30 分钟。
3. **标定板**：保持已确认的准确理论尺寸；板平整、固定在工作台上。
4. **相机**：在 Box 上运行 `tools/zed_camera_yaml.py --output camera.yaml`，与采集程序的校正后左目内参和配置核对（见 2.1）；不要直接比对 Explorer 的 raw 内参。确认存下的图像是**校正后的单张左目图**：不是左右拼接图，不是原始未校正图，PNG 格式，1920×1200，`FLIP_MODE.OFF`。
5. **单张图**：`detect --overlay`，应检出 36 个 tag，蓝色 `0` 在 tag 的物理右下角。
6. **位姿**：确认 WebApp 当前的工具坐标系和工件坐标系都是 0；记录 8 个以上分散的姿态（附关节角），运行 `check-poses`，应判 `pass`，拟合出的偏移应接近 0。
7. **标定**：采集 20–30 个姿态，带上支架 CAD 平移运行 `solve`。PnP 误差预计在 0.5 px 以下，然后逐条看警告。
8. **重复性**：换一组姿态再标一次，用 `compare` 比较，应判 `CONSISTENT`。
9. **独立验证**（有尖端工具时）：另采未用于拟合的 session，运行 `validate --session`；不加 `--session` 仅检查板位姿。
10. **调整门限**：根据前几次的实际数值，调整质量门限和诊断阈值。

## 坐标约定与依据

- FR5 姿态：[FAIRINO 手册](https://fairino-doc-en.readthedocs.io/latest/CobotsManual/robot_brief_introduction.html)规定为移动轴 ZYX，等价于固定轴 `R = Rz(rz)·Ry(ry)·Rx(rx)`。FR5 的 DH 参数取自同一手册，已与官方 URDF 核对（差 ≤ 0.1 mm）。
- 标定板：[Kalibr 板生成器](https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_create_target_pdf)使用 AprilTag 36h11、2 码元黑边和旋转 180° 的码图；[Kalibr 坐标定义](https://github.com/ethz-asl/kalibr/blob/master/aslam_cv/aslam_cameras_april/src/GridCalibrationTargetAprilgrid.cpp)以 tag 0 左下角为原点，+x 向右，+y 向上。已用整板照片验证 ID 0–35 和角点方向。
- 结果是"ZED 校正后左目光学坐标"到法兰坐标的变换，不是相机外壳中心、右目或工具 TCP 的变换；左目光心位于相机外壳横向中心的 −x 方向 60 mm 处。
- 旋转向量统一用 `geometry.rotation_vector` 计算：`cv2.Rodrigues` 会把小于约 1e-5 rad 的旋转直接当成 0。
