# 固定 cam0 与双 UR 手眼标定：当前 ChArUco 板

项目：`/home/descfly/hmj/ur`，脚本：`tools/calibrate_ur_cam0_aruco.py`。
这是固定相机、标定板固定末端的 eye-to-hand 标定。脚本只读图像、CameraInfo 和图像时刻的 TF，不控制机械臂或手。

## 当前板的实际参数

用户于 2026-10-22 确认：棋盘方格边长 **20 mm**，方格内 ArUco 码黑色外框边长 **15 mm**。

| 字段 | 当前值 |
| --- | --- |
| 类型 | ChArUco |
| 字典 | DICT_5X5_250，照片中的 ID 0–62 均成功解码 |
| 列数 × 行数（按板的编号方向） | **14 × 9 个方格** |
| 总码数 / 内部棋盘交点数 | 63 / 104 |
| square_length_m / marker_length_m | 0.020 / 0.015 |
| legacy_pattern | true，固定跨版本棋盘布局 |
| 最少可见码 / 最少有效交点 | 5 / 12 |
| 单帧重投影 RMSE 上限 | 3 px |

照片竖放看起来是9列14行，但 ID 排列对应 **14列9行的板旋转90°**。不能仅按照片横竖交换配置行列，否则ID对应关系错误。板可以任意旋转摆放，配置不随摆放方向改变。
`ids_row_major` 按板本身编号方向、从左到右和从上到下列出有标记的白格，当前为0–62。
字典前缀可能共用，但本板全部63个码已用DICT_5X5_250解码并核对空间排列。

坐标原点在整个棋盘区域的左上外角，不是纸张或外侧白边的角；x沿14个方格方向，y沿9个方格方向，z按右手系。原点及轴由预览中的坐标轴确认。板棋盘区物理尺寸为280×180 mm。

左右配置分别是：

- `robot_control/config/left_cam0_charuco_handeye.json`
- `robot_control/config/right_cam0_charuco_handeye.json`

旧3×3 GridBoard配置和旧结果保留。所有命令显式指定新配置，省略 `--config` 仍使用旧的右臂GridBoard配置。

## 1. 启动 UI 和环境

VS Code 打开项目：

```bash
code /home/descfly/hmj/ur
```

终端一：

```bash
cd /home/descfly/hmj/ur
./start_dual_ur_ui.sh
```

在UI开启cam0及双臂Driver。停止模型部署、回放等其他运动程序；用UI、GELLO或示教器操作当前被标定臂。整个过程保持cam0和双臂基座不动。

终端二，后续所有标定命令均在这个终端执行：

```bash
cd /home/descfly/hmj/ur
source ./setup_env.sh
ros2 topic echo /cam0/color/camera_info --once
```

## 2. 先左臂：预览、采样、求解

将板牢固固定在左臂末端，不能在采样或该侧独立验证中改变板与末端的安装关系。若两臂使用相同ID的板，只让当前被标定的一块进入视野。

预览：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py preview \
  --config robot_control/config/left_cam0_charuco_handeye.json
```

应显示码的边框、棋盘交点和坐标轴，并出现 `ChArUco OK`。尽量让整块板清晰可见，避免反光、模糊。Q或ESC退出。

采集：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py collect \
  --config robot_control/config/left_cam0_charuco_handeye.json
```

采约25–30组：移动左臂→停稳至少1秒→点击图像窗口→按空格→确认终端“已保存第N组”。
覆盖实际工作区不同位置、距离，并绕至少两个方向改变倾角约20–40°，不能只平移或只绕一个轴转。至少12组才能求解。
Q或ESC结束，再求解：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py solve \
  --config robot_control/config/left_cam0_charuco_handeye.json
```

默认结果：

```text
/home/descfly/hmj/ur/grasp/config/ur_cam0_charuco_calibration/left/
├── samples.json
├── sample_001.png ...
└── calibration.json
```

样本保存可见码ID、ChArUco交点ID、对应米制三维点及图像坐标、图像时间戳、图像时刻的TF和重投影误差。
PNG为带检测标记的检查图。JSON中的角点来自原始输入图像；求解使用JSON中的位姿。

## 3. 左臂独立验证（先验证，再拆板）

板安装及相机均保持不变，另采约10组没有参与拟合的新姿态。验证数据不能与拟合数据使用同一个目录；验证目录已有数据时用新的名称。

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py collect \
  --config robot_control/config/left_cam0_charuco_handeye.json \
  --output-dir grasp/config/ur_cam0_charuco_calibration/left_validation

./.venv/bin/python tools/calibrate_ur_cam0_aruco.py validate \
  --config robot_control/config/left_cam0_charuco_handeye.json \
  --output-dir grasp/config/ur_cam0_charuco_calibration/left_validation \
  --calibration grasp/config/ur_cam0_charuco_calibration/left/calibration.json
```

不对验证数据运行solve。结果为验证目录下的`validation.json`，不会重拟合已有矩阵。

## 4. 再右臂

保持cam0和基座不动，将板固定到右臂。重复预览、采样和求解：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py preview \
  --config robot_control/config/right_cam0_charuco_handeye.json

./.venv/bin/python tools/calibrate_ur_cam0_aruco.py collect \
  --config robot_control/config/right_cam0_charuco_handeye.json

./.venv/bin/python tools/calibrate_ur_cam0_aruco.py solve \
  --config robot_control/config/right_cam0_charuco_handeye.json
```

右臂结果默认保存在 `grasp/config/ur_cam0_charuco_calibration/right/`。

保持右臂板的安装不变，独立验证：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py collect \
  --config robot_control/config/right_cam0_charuco_handeye.json \
  --output-dir grasp/config/ur_cam0_charuco_calibration/right_validation

./.venv/bin/python tools/calibrate_ur_cam0_aruco.py validate \
  --config robot_control/config/right_cam0_charuco_handeye.json \
  --output-dir grasp/config/ur_cam0_charuco_calibration/right_validation \
  --calibration grasp/config/ur_cam0_charuco_calibration/right/calibration.json
```

## 5. 质量门槛和结果方向

保持原有保护：按图像时间戳取TF；CameraInfo/分辨率/坐标系一致；停稳采样；近重复姿态拒绝；重投影RMSE≤3px。
ChArUco交点不足、共线或位姿歧义时拒绝，不退回用标记角点代替。
每第5个姿态留出验证，最后用全体样本重拟合。

全样本、集合内留出及独立验证分别检查：位置RMSE≤10mm、最大≤10mm、旋转RMSE≤2°、最大≤5°。
未通过仍保存结果并返回退出码2，不能将其当作通过。预览/采集故障返回1。

结果满足：

```text
T_base_tool @ T_tool_board = T_base_cam0 @ T_cam_board
p_base = T_base_cam0 @ p_cam0_optical
```

各臂`calibration.json`包含`T_base_cam0`和`T_cam0_base`及对应侧别名。平移及输入三维点单位为米。
使用`left_base`/`right_base`和`tool0`，不能直接替换成`base_link`或铲子TCP。脚本不发布TF、不自动修改部署配置，也不自动导出双臂合并文件。

板重新安装或cam0/基座移动后，使用新的`--output-dir`；求解时使用同一目录。不要追加到旧采样中。改变配置会拒绝混用，机械安装变化也必须由操作者分开目录。
标定残差不是实际TCP接触精度保证；仍需核对实际工具偏移、深度与坐标链路。
