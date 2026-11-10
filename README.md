# 两套设备共用任务标定板：观测与坐标对齐

两套设备不在同一场地时，分别把同样的板放在操作区的**对应位置和方向**，工具计算每台相机到板的位姿，再建立任务迁移关系。相机视角无需完全一样。

**如果已有手眼标定，保持相机和机械臂底座的相对位姿。两者的相对位姿发生变化后，需要重新确认手眼外参。**

本工具只读取图像并保存文件，不向机械臂发送指令，也不修改已有标定。尚未用你的真实新设备采集验证；示例目录中的图像与位姿均为合成测试。

## 先把板摆对

默认配置来自原 2026-10-30 手眼标定：ChArUco，14×9 格，格长 20 mm，标记边长 15 mm，字典 `DICT_5X5_250`。有效板面为 280×180 mm，默认编号从 0 开始。

1. 对照 `board/board_reference.png` 检查实物板图案、黑白排列、方向和编号；仅格子尺寸相同不能保证是同一布局。若现有板不一致，先修改 `config/board.json` 或使用提供的板。
2. 需要打印时，使用 `board/board_actual_size.svg`，按 **100% / 实际大小** 打印，不要“适合页面”。用尺确认每格 20 mm，板面 280×180 mm；贴在平整硬板上。
3. 在两套操作区分别确定同一个任务基准。例如，以托盘摆放中心为基准，记录板左上角相对它的前后、左右距离，板的一条边与桌面基准边平行。使用定位边、胶带标记或夹具复现位置。板厚、桌面高度关系也要一致。
4. 两处板的“左上角”必须对应，不能把板旋转 180° 后仍沿用原摆放定义。建议在硬板边缘标记原点、X 箭头和 Y 箭头。
5. 拍摄时板和相机都保持不动，避开反光，尽量让整块板可见。实时模式会收集 30 帧稳定结果。

板坐标系固定为参考图的**左上外角原点、X 向图案右侧、Y 向图案下侧、Z 指向板内**。板平放时 Z 通常朝桌内，不是重力向上。程序固定角点 ID 到坐标的关系，并已检查 OpenCV 4.6 / 4.11 的一致性。

两地的物体、托盘相对板也要对应。仅让板在图像中的位置接近，不能保证它们在操作区的位置对应；本工具不会自动恢复原录像中物体的摆放位置。

## 使用环境

推荐 Python 3.10+。普通照片或 USB 模式可在独立虚拟环境中安装：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

ROS 2 模式优先用 ROS 自带的系统 Python、`cv_bridge` 和 OpenCV，避免混用 Conda 与 ROS 的依赖。本机已验证系统 OpenCV 4.6.0 和独立环境 OpenCV 4.11.0。

下面命令均在代码包根目录执行。TRON2 的 ROS 2 Foxy 兼容入口和 franka-sharpa 对齐命令见 [工位说明](ZHUJI.md)。

## 方法一：ROS 2 实时观测

使用同一彩色相机的原始 `sensor_msgs/Image` 和配套 `sensor_msgs/CameraInfo`，不是深度图或另一只相机的 CameraInfo。替换命令中的话题名。

原设备：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 observe_ros2.py \
  --image-topic /your_color/image_raw \
  --info-topic /your_color/camera_info \
  --camera-id source_room \
  --out results/source --show --manual-save --timeout 600
```

画面显示坐标轴、识别角点、重投影误差。保持不动，等待稳定后按 **S** 保存，按 **Q** 退出。无显示器时去掉 `--show --manual-save`，达到稳定条件后自动保存。

把 `results/source/observation.json` 复制到新设备上，例如放在同名目录。新设备执行：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 observe_ros2.py \
  --image-topic /your_color/image_raw \
  --info-topic /your_color/camera_info \
  --camera-id target_room \
  --reference results/source/observation.json \
  --out results/target --show --manual-save --timeout 600
```

新设备画面额外显示：相机相对板的位置差（板坐标系下的毫米值）和朝向差。它们用于比较摆放，**不要求调到零**。不要为消除差值移动已经做过手眼标定的相机；固定相机后正确摆放任务板即可。

默认处理未去畸变的彩色原图，使用 CameraInfo 的 K/D。若确实使用已校正图像，可加 `--rectified` 使用 P 和零畸变；只支持 R 为单位矩阵且无立体投影偏移的情况。非单位 R、裁剪、binning、鱼眼模型会拒绝，避免混用坐标系。

## 方法二：保存照片后观测

每台相机用自己的内参，且图像分辨率、裁剪和去畸变状态必须匹配。支持普通 JSON、ROS 相机标定 YAML，以及本项目旧 `handeye_calibration.json` 的 `camera_intrinsics` 字段。

两台相机分别使用各自当前彩色图像配置对应的内参。

```bash
python3 board_align.py observe \
  --images /path/to/source_board.png \
  --intrinsics /path/to/source_camera_intrinsics.json \
  --camera-id source_room --out results/source

python3 board_align.py observe \
  --images /path/to/target_board.png \
  --intrinsics /path/to/target_camera_intrinsics.json \
  --reference results/source/observation.json \
  --camera-id target_room --out results/target
```

`--images` 可跟多张照片，但所有照片必须在相机和板均固定时拍摄。工具拒绝尺寸不匹配、未找到板、角点太少、角点覆盖太窄、误差过大或多张照片位姿不稳定的情况。单张照片可求位姿，但不能检查时间稳定性。

内参 JSON 示例字段如下；数值必须填你自己的标定结果：

```json
{
  "width": 1280,
  "height": 800,
  "K": [[1050, 0, 640], [0, 1040, 400], [0, 0, 1]],
  "D": [0, 0, 0, 0, 0],
  "distortion_model": "plumb_bob",
  "frame_id": "camera_color_optical_frame"
}
```

## 方法三：USB / OpenCV 摄像头

需要事先取得当前分辨率下的该摄像头内参，普通 USB 设备不会自动提供标定内参。

```bash
python3 board_align.py live --device 0 \
  --intrinsics /path/to/camera_intrinsics.json \
  --camera-id target_room --out results/target \
  --reference results/source/observation.json \
  --show --manual-save --timeout 600
```

## 生成两套场景的转换

把两个设备的结果复制到同一台电脑，再执行：

```bash
python3 board_align.py align \
  --source results/source/observation.json \
  --target results/target/observation.json \
  --out results/task_mapping.json
```

输出 `target_camera_T_source_camera`。列向量、米单位：

```text
T_target_camera_source_camera = T_target_camera_board @ inverse(T_source_camera_board)
T_target_camera_wuji = T_target_camera_source_camera @ T_source_camera_wuji
T_target_base_wuji = T_target_base_target_camera @ T_target_camera_wuji
```

`T_target_base_target_camera` 来自新设备已有的手眼标定。左右手使用同一个任务映射，然后分别转入对应机械臂底座系。若 EEF 与 Wuji 手根的原点和轴都一致，末式就是 EEF 目标；否则还需乘固定工具变换的逆。

当前转换包的 `wuji_motion_scene.npz` 属于原装配模型世界系，**不能直接当成 `T_source_camera_wuji`**；需要先借助原手眼标定转换到原相机系。已有的左手 +90°、右手 −90° 修正不要重复添加。本工具目前只生成板观测与场景映射，不会直接修改轨迹、求机械臂 IK 或执行机器人。

这里定义的是两地**对应任务坐标系**之间的映射，不是测量两栋房间中相机的物理距离。若板在两处相对物体有 1 cm 摆放差，任务映射也会带入相应偏差。

## 已归档观测与新增设备

当前实机板观测位于 [observations/](observations/)。[zhuji](observations/zhuji/observation.json) 和 [franka-sharpa](observations/franka-sharpa/observation.json) 各有一份观测。等待其他设备采集完成后统一计算映射。

### 新设备交付清单

1. 为设备选定一个唯一名称，例如 `robot-c`。沿用同一块板或同款板，按相对各自托盘或工作台的对应位置、朝向和图案面高度摆放。板和相机在本次采集期间保持固定。若相机装在可动机构上，记录采集姿态，以便复现。
2. 用该相机自己的彩色原图和配套内参采集。ROS 2 采集用 `--camera-id robot-c --out results/robot-c`，默认保存 30 帧稳定观测。照片模式交付原始彩色图和同分辨率的内参（宽高、K、D、畸变模型和光学坐标系），由脚本生成同格式的观测。
3. **每台设备必交一份 `observation.json`。** 文件应包含设备名、板型及其指纹、`camera_T_board` 和逆变换、相机内参、样本数与质量统计。归档路径为 `observations/<设备名>/observation.json`。原图 `image.png`、角点叠加图 `overlay.jpg` 和逐帧 `samples.json` 留在采集主机用于复核。
4. 如果最终还要把坐标转换到某台机器人的基座，另外准备该设备与采集姿态匹配的相机到基座外参，并注明坐标系和姿态。只对齐相机光学坐标系时，各设备的板观测即可。

## 保存的文件与检查

每次观测生成：

- `observation.json`：板到相机位姿、逆变换、相机内参、板配置、样本数量和质量统计。
- `samples.json`：每帧位姿、角点和误差，便于复核。
- `image.png`：最后一帧原图。
- `overlay.jpg`：识别角点、坐标轴、误差与可选参考比较。

实时默认要求 30 帧、每帧至少 12 个有效角点、重投影 RMS 不超过 1.5 像素；样本相对代表位姿的最大位置散布不超过 3 mm，旋转散布不超过 1°。这些是内部一致性检查，**不代表已验证实际绝对定位精度**。

合成测试覆盖已知位姿、跨版本板图案与坐标约定、映射方向、无板/尺寸错误/配置不一致的拒绝和稳定窗口检查。另外已通过带畸变的合成图像测试，以及仅限本机独立 ROS 域的 Image + CameraInfo 到观测保存的端到端测试（6 帧）。详见 `validation.json`。`example_synthetic/` 只用于展示，不能作为真实设备的标定结果。

实现参考：[OpenCV ChArUco](https://docs.opencv.org/4.8.0/d0/d3c/classcv_1_1aruco_1_1CharucoBoard.html)、[OpenCV PnP 位姿定义](https://docs.opencv.org/4.8.0/d5/d1f/calib3d_solvePnP.html)。
