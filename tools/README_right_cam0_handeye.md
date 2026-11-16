# 左右 UR 基座与固定 cam0 的手眼标定（先左后右）

本文针对旧3×3 ArUco GridBoard。2026-10-22的新ChArUco板（20mm方格、15mm码边长）
请使用 [ChArUco标定说明](README_cam0_charuco_handeye.md) 和其中的左右臂新配置。

脚本：`tools/calibrate_ur_cam0_aruco.py`。左右配置分别为：

- `robot_control/config/left_cam0_handeye.json`
- `robot_control/config/right_cam0_handeye.json`

这是 eye-to-hand：cam0 固定，GridBoard 刚性固定在当前被标定机械臂末端。
脚本只订阅彩色图、CameraInfo 和 TF，不控制机械臂或灵巧手，不需要深度图。
用已有 UI / GELLO / 示教器移动机械臂。

## 1. 核对标定板

当前两份配置均为：

| 参数 | 配置 |
| --- | --- |
| 字典 | DICT_4X4_250 |
| 列数×行数 | 3×3 |
| 单个码黑色正方形边长 | 35mm，即0.035m |
| 相邻黑方块间的白色净间隔 | 6mm，即0.006m |
| 最低可见码数量 | 5个，尽量完整看到9个 |
| ID，从左到右、从上到下 | 0、1、2 / 3、4、5 / 6、7、8 |

**上述尺寸已由用户测量实物确认；更换标定板后必须重新测量。**
测量的是黑色正方形的外侧边长（包含编码外黑框、不包含外围白边），不是整张纸。
按此尺寸，3×3板最外侧黑边间总宽高均为117mm，不包括外围白边。
若实物间隔或ID排列不同，必须同时修改两份配置的 `marker_separation_m` 或 `ids_row_major`。
图片像素大小不是物理尺寸，打印不能非等比拉伸；各码不得单独旋转。

将板贴在平整、不反光的硬板上，牢固连接当前机械臂末端。板相对 `left_tool0` 或
`right_tool0` 必须保持不动；如果用手指夹持，整个采集过程不能改变抓握姿态。
左右两块板可以使用相同ID，但采集时只能让其中一块进入视野，否则会出现重复ID。
也可以用同一块板，完成左臂后拆下并重新固定到右臂，左右采样分开保存。
**两臂标定之间不能移动cam0。**

## 2. 启动UI与检查

终端一：

```bash
cd /home/descfly/hmj/ur
./start_dual_ur_ui.sh
```

在UI中打开cam0和左臂Driver；标定右臂时再确认右臂Driver已开启。
终端二：

```bash
cd /home/descfly/hmj/ur
source ./setup_env.sh
ros2 topic echo /cam0/color/camera_info --once
ros2 run tf2_ros tf2_echo left_base left_tool0
```

TF应持续输出。按Ctrl+C结束检查，然后运行下面的预览。
默认ROS域为44；相机和Driver须在同一ROS域，Driver须使用对应机械臂的出厂运动学标定参数。
`left_base` / `right_base` 采用UR控制器基座约定，不要随意替换成 `*_base_link`
（可能绕Z轴相差180°）。`*_tool0` 不是任意配置的TCP。

## 3. 先标定左臂

预览：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py preview \
  --config robot_control/config/left_cam0_handeye.json
```

核对检测出的实际ID与上述排列一致，出现坐标轴及 `Board OK`。
点一下图像窗口使其获得焦点，按Q或ESC退出。
板原点在左上角第一个码的左上黑角，x向板右、y向板下、z按右手系指向板背面。
cam0 optical frame为x向图像右、y向下、z向前。

开始采集：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py collect \
  --config robot_control/config/left_cam0_handeye.json
```

每个姿态：移动左臂 → 停稳至少1秒 → 点击图像窗口 → 按空格 → 确认终端“已保存第N组”。
采约25组，覆盖工作区不同位置与距离，并绕至少两个不同轴改变约20–40°。
不能只平移或只绕一个轴转。保持标记清晰可见，避免反光、遮挡和模糊。
采集完成在图像窗口按Q或ESC退出。

求解：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py solve \
  --config robot_control/config/left_cam0_handeye.json
```

左臂输出：

```text
/home/descfly/hmj/ur/grasp/config/ur_cam0_calibration/left/
├── samples.json
├── sample_001.png ...
└── calibration.json
```

检查 `quality_pass`。失败时先处理原因并补采或重新采集，再进行使用。

## 4. 再标定右臂

保持cam0不动，将板牢固安装在右臂，遮住/移出左臂同ID的板。
先检查右臂TF，确认后按Ctrl+C结束检查：

```bash
ros2 run tf2_ros tf2_echo right_base right_tool0
```

依次执行预览、采集和求解，操作与左臂相同：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py preview \
  --config robot_control/config/right_cam0_handeye.json
```

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py collect \
  --config robot_control/config/right_cam0_handeye.json
```

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py solve \
  --config robot_control/config/right_cam0_handeye.json
```

右臂输出：`/home/descfly/hmj/ur/grasp/config/ur_cam0_calibration/right/`。
脚本默认配置仍为右臂；为避免选错，建议始终像上面一样显式指定 `--config`。

## 5. 样本保护与质量检查

- 使用图像时间戳对应的TF，不用最新TF替代；图像陈旧超过1秒会拒绝。
- Image与CameraInfo必须在同一彩色光学坐标系，分辨率、内参有效且一致。
- 需至少0.5秒稳定观测：末端变化≤1mm/0.5°，板视觉位姿变化≤5mm/2°。
- 每组角点重投影RMSE≤3px；连续近重复姿态不保存。
- 角点细化窗口为2px，避免小间隔标记的相邻角点被默认5px窗口合并。
- 每组成功后立即保存，再次启动可追加；配置、内参或坐标系不一致时拒绝混用。
- 至少12组才能求解，建议25组；每第5组留出验证，然后用全部样本重拟合。
- 全样本和留出验证均要求：平移RMSE≤10mm、最大≤10mm；旋转RMSE≤2°、最大≤5°。
- 未通过也保存诊断结果，退出码为2。查看 `sample_errors` 排查异常。

相机或板重新安装后，即使配置文字相同也不能继续追加旧样本，应换新目录，例如：

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py collect \
  --config robot_control/config/left_cam0_handeye.json \
  --output-dir /home/descfly/hmj/ur/grasp/config/ur_cam0_calibration/left_session2
```

求解时须指定相同的 `--output-dir`。更改板配置之前已经采集的旧单码样本不会被自动覆盖或混用。

## 6. 结果方向与使用

同时求解相机到基座、标定板到末端的安装偏移，无需手量板到末端偏移：

```text
T_base_tool × T_tool_board = T_base_cam0 × T_cam_board
```

每份结果都含通用字段 `T_base_cam0`、逆变换 `T_cam0_base`；
左臂另外保存 `T_left_base_cam0` / `T_cam0_left_base`，
右臂另外保存 `T_right_base_cam0` / `T_cam0_right_base`。
`parent_frame` 指明实际UR基座，`child_frame` 指明cam0彩色光学坐标系。

```python
import json
import numpy as np

path = "/home/descfly/hmj/ur/grasp/config/ur_cam0_calibration/left/calibration.json"
with open(path) as f:
    result = json.load(f)
assert result["quality_pass"], "标定质量检查未通过"
assert result["parent_frame"] == "left_base"
T = np.asarray(result["T_base_cam0"])
# 示例：真实米制三维点，不是图像像素。
p_cam0 = np.array([0.1, 0.0, 0.8, 1.0])
p_left_base = (T @ p_cam0)[:3]
```

脚本不发布TF，也不修改部署配置。`quality_pass: true`只代表内部检查通过；
用于机器人运动前，另用未采集姿态或已知参考点独立核对定位精度。
打印尺寸、内参和机械臂运动学的系统误差可能无法通过低拟合残差发现。

## 7. 用新姿态独立验证右臂

完成右臂标定后，cam0、标定板与右臂末端的固定关系均不能改变。
另采约10组未参与拟合的新姿态，覆盖实际工作区近处、远处、四周和不同倾角。
使用单独目录；如果该目录已有上次验证样本，请改用新的目录名。

```bash
./.venv/bin/python tools/calibrate_ur_cam0_aruco.py collect \
  --config robot_control/config/right_cam0_handeye.json \
  --output-dir grasp/config/ur_cam0_calibration/right_validation

./.venv/bin/python tools/calibrate_ur_cam0_aruco.py validate \
  --config robot_control/config/right_cam0_handeye.json \
  --output-dir grasp/config/ur_cam0_calibration/right_validation \
  --calibration grasp/config/ur_cam0_calibration/right/calibration.json
```

不要对验证集运行solve。validate保持原来的相机和板安装变换不变，比较：
相机观测推算的板原点基座位置，与UR末端状态推算的板原点基座位置。
终端逐组打印三维位置距离（mm）及角度误差，并保存validation.json。
要求验证样本的平移RMSE、最大误差均≤10mm，角度及图像重投影标准同上。
原标定quality_pass和独立validation_pass分别报告，验证不会把原先失败的标定改成通过。
这些是板原点的一致性误差；还需用已知参考点检查实际使用的TCP、深度和坐标转换链路。
