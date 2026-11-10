# 双相机对齐在 zhuji 上怎么做

这里以 franka-sharpa 工位的 Orbbec Gemini 435Le 为源相机，以 TRON2 头部 RealSense 为目标相机。每一路使用自己的彩色图像和配套内参。

1. 两个工位各用相同图案的 ChArUco 板，14×9 格、每格 20 mm、标记 15 mm、字典 `DICT_5X5_250`。用 `board/board_reference.png` 确认图案和编号。打印 `board/board_actual_size.svg` 时用实际大小，量一下板面是否为 280×180 mm。
2. 以各自的工作台或托盘为基准，把板原点放在对应位置，X/Y 方向和板面高度也保持对应。相机视角可以不同。采样时固定板和相机。TRON2 头部角度应与后续使用时一致。如果要沿用约 41.5° 对应的基座外参，先用历史实测姿态复位并记录反馈。那份基座外参仍是运动学估计。
3. 在 franka-sharpa 工位采集 `results/franka-sharpa/observation.json`。使用导入包的 [README](README.md) 中 ROS 2 或照片模式，参数用 `--camera-id franka-sharpa --out results/franka-sharpa`，并确认源工位当前的彩色图与内参匹配。如果更改了彩色图的分辨率或裁剪，重新获取与之匹配的内参。把源侧观测复制到 zhuji 的本目录。
4. 板摆好后在 zhuji 上采 TRON2 目标侧。脚本用彩色话题的在线内参，默认采到 30 帧稳定观测后保存：

```bash
cd /path/to/calibration
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/ros-bringup/fastdds_config_udp_only.xml

/usr/bin/python3 zhuji_board_align.py ros \
  --image-topic /camera/top/color/image_raw \
  --info-topic /camera/top/color/camera_info \
  --camera-id tron2_top --out results/target --timeout 120
```

5. 两侧真实观测齐备后生成从源 RGB 光学坐标到目标 RGB 光学坐标的矩阵：

```bash
/usr/bin/python3 zhuji_board_align.py align \
  --source results/franka-sharpa/observation.json \
  --target results/target/observation.json \
  --out results/task_mapping.json
```

输出 `target_camera_T_source_camera`，单位米，使用列向量。先看两侧 `overlay.jpg` 的板轴和角点，再用板外另一个已知位置做一次独立核对。如果还要变换到 TRON2 基座，另乘与采集姿态一致的 RGB 到基座外参。

zhuji 系统 Python 配套 ROS 2 Foxy 的 `cv_bridge`。本机 OpenCV 4.2.0 的角点 ID 从板的下边开始数行，`zhuji_board_align.py` 仅对这个版本和原配 14×9 板做行编号换算，沿用原包的位姿计算。用本机运行时请通过此入口采集。附件原始脚本和样例均保留。
