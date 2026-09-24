# 开发与交接说明

给接手维护的人（或编码助手）看。使用方法见 [README.md](README.md)；这里只记录代码里看不出来的东西：边界、约定、设计原因、已知问题。

## 1. 项目定位与边界

- 本仓库**只做手眼标定算法**：输入是 session 文件夹（`camera.yaml`、`poses.csv`、`images/`、`target.yaml`），输出是 `result.json`。
- **数据采集不在本仓库**。硬件是：ZED Box Mini 和 ZED X 装在 FR5 末端，Box 接显示器保存图像；笔记本用网线连接 FR5，通过 WebApp 移动机械臂、抄录位姿和关节角。
- 核心代码**不依赖 ZED 或 FAIRINO 的 SDK**，不要把采集或 SDK 调用加回 `handeye/`。唯一的例外是 `tools/zed_camera_yaml.py`，在 Box 上运行，用来导出内参。
- 早先基于 SDK 的自动采集代码：较早的版本在 git 历史里（提交 `4481b40` 的 `handeye.py`）；后来改进的 `acquisition.py`（按 `frame_cnt` 检查数据新鲜度）和 `box_sender.py` 从未提交，只保存在本地的 `calibration-data/archive/`。
- 设备状态：截至 2026-09-24 **还没有用真机数据跑过**，全部验证都来自仿真和单元测试。

## 2. 代码结构与数据流

```text
python -m handeye solve
  dataset.load_session        读 camera.yaml / poses.csv / target.yaml
  observations.build_...      每张图：board.detect_board → PnP → 剔除锁错的 tag
  solver.calibrate
    kinematics.row_consistency   有关节角时：逐行核对位姿，拟合 TCP / 工件坐标系偏移
    diagnostics.coverage         覆盖度
    diagnostics.intrinsics_from_images   只用图像检查 K 和 k1
    check_motion_diversity       旋转轴不足 → insufficient_data
    fit_candidates + cross_validated_rmse   Park / Daniilidis / 像素优化，4 折 CV 选择
    quality_gate                 测试视图门限、CAD 核对、TCP 拒绝
    diagnostics.bootstrap        jackknife 不确定度
    diagnostics.board_scale      结合机械臂数据拟合板尺度
  __main__.write_json          result.json（inf 写成 null）
```

- `geometry.py`：变换、FR5 欧拉角、`rotation_vector`、Unicode 安全的图像读写。
- `kinematics.py`：FR5 的 DH 模型、关节限位、数值逆解（仿真用），以及位姿与关节角的检查。
- `validation.py`：尖端点验证（`validate`），以及多次标定结果的比较（`compare`）。
- `sim/synthetic.py`：生成与真实数据同格式的合成 session（另附 `truth.json`），`suite` 是回归检查。

## 3. 必须保持的约定

改动下面任何一项之前，先在仿真和测试里证明新做法更好。

- **坐标系命名**：`A_T_B` 把 B 中的点变换到 A。内部一律用米和弧度；FR5 的读数是毫米和度。
- **FR5 姿态**：`R = Rz(rz)·Ry(ry)·Rx(rx)`（手册写的是移动轴 ZYX）。位姿指**法兰**（工具坐标系 0）在**基坐标系**（工件坐标系 0）下的值。
- **Kalibr 板**：AprilTag 36h11，`markerBorderBits=2`，码图旋转 180°，因此 OpenCV 的角点 0..3 依次对应 tag 的物理 BR、BL、TL、TR 角；原点在 tag 0 的左下角。这已用实物照片验证过，不要"修正"。
- **图像**：ZED 校正后左目、`FLIP_MODE.OFF`、内参取校正后的 `left_cam`。翻转 180° **任何检查都发现不了**。
- **旋转向量**：一律用 `geometry.rotation_vector`，不要用 `cv2.Rodrigues(矩阵)`。后者会把小于约 1e-5 rad 的旋转当成 0，曾导致逆解和残差看不见这么小的误差。
- **图像读写**：一律用 `read_image` / `write_image`。仓库在中文路径下，`cv2.imread` / `cv2.imwrite` 读写不了。
- **测试视图**：每 5 个可用视图取 1 个作为测试视图，只用于报告和质量门限，**绝不参与选方法**。选方法只看训练视图上的 4 折交叉验证。
- **诊断与判定**：诊断只发警告。只有质量门限、CAD 偏差、位姿读成 TCP 这几项会导致 `rejected`。门限都是项目默认值，不是厂家指标。

## 4. 关键设计决策及原因

| 决策 | 原因（均有仿真证据） |
|---|---|
| 检测后再按间隙大小做一次 `cornerSubPix`，并按整板残差剔除 tag | Kalibr 板上的小方块让 tag 角变成 X 形交点，OpenCV 在这里偏 1–4 px，偶尔十几像素；二次精化后平均误差 0.03 px |
| 内参和畸变只用图像检查（`calibrateCameraExtended`，每张图的板位姿各自自由） | 和机械臂数据一起拟合时，0.5 mm 的位姿噪声被当成了 2.7 px 的畸变，正常数据也误报；只看图像时机械臂误差漏不进来 |
| 只拟合 k1，不拟合 k2 | k1、k2 同时放开时，外推到图像角落的结果不稳定 |
| 板尺度结合机械臂拟合，内参固定为 `camera.yaml` | 单看图像无法确定尺度；位姿噪声 0.2–0.5 mm 时，估计只偏不到 0.1% |
| 不确定度用 delete-d jackknife，不用雅可比 | 雅可比只计入像素噪声，把真实误差低估约 10 倍；jackknife 与真实误差在同一量级 |
| 位姿读成 TCP 直接拒绝 | 否则会悄悄接受一个偏了 120 mm（等于 TCP 偏移）的结果 |
| 工件坐标系只警告 | 手眼结果不受基座端偏移影响 |
| 坐标系偏移只在旋转足够分散时报告 | 姿态单一时偏移本身拟合不准，会产生误报 |
| 用交叉验证选方法 | 像素优化在机械臂噪声大时可能反而更差，CV 会自动选 Park 或 Daniilidis |

硬件方面的审查意见和早期讨论记录在本地的 `calibration-data/reference/HARDWARE_REVIEW.md`（描述的是改版前的采集方案，仅作背景参考）。

## 5. 开发流程

- 在仓库根目录运行所有命令。环境：`python -m venv .venv` 后安装 `requirements.txt`；测试过 Python 3.10 和 3.14（版本表见 README 1.4 节）。
- **单元测试**：`python -B -m unittest discover -s tests`，35 个，约 1 分钟，必须全部通过。
- **仿真回归**：`python -B sim/synthetic.py suite`，约 2.5 分钟，12 个检查场景必须全部符合预期（否则退出码 1）。数值变化后，同步更新 README 第 3 节和 4.6 节的表格。
- **提交**：由用户手动提交。不要自行 commit、push 或改动暂存区。
- **不要生成 `__pycache__`**：命令都带 `-B`；`sim/` 里也设置了 `sys.dont_write_bytecode`。
- **本地规则**（`AGENTS.md`，不进 git）：不启用 trace 级日志或 trace 导出；避免持续、高频的磁盘写入。
- **本地数据**都放在 `calibration-data/`（已被 git 忽略）：`sessions/` 放真实数据，`sim_runs/` 放仿真输出，`reference/` 放数据手册和审查报告，`archive/` 放旧的采集代码。

## 6. 已知局限与待办

1. **真机未验证**：真实图像的检测质量、真实内参精度、FR5 的绝对精度、门限的合理取值，都要按 README 第 5 节的试标清单去确认；Ubuntu 上也还没跑过测试。
2. **`tools/zed_camera_yaml.py` 未在硬件上运行过**：pyzed 的属性名以实际 SDK 版本为准。
3. **图像翻转发现不了**：只能靠相机设置 `FLIP_MODE.OFF` 预防。
4. **仿真的局限**：渲染与检测共用同一板定义；内参为名义值；FR5 只用名义运动学，未检查碰撞；有损视频只用 JPEG 近似。
5. **机械臂位姿视为准确**：没有建立机械臂误差模型，这方面的误差只通过 jackknife、测试视图和 `compare` 间接反映。
6. **覆盖度警告的阈值**（图像覆盖 50%）在仿真里总会触发，需要根据真实数据调整。
7. **可以考虑的改进**（尚未做）：把关节零位误差纳入联合优化；用真实图像校准诊断阈值；如果 FR5 的绝对精度成为瓶颈，考虑做运动学标定。
