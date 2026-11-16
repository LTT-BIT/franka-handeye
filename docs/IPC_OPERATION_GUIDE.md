# 工控机完整操作手册：Franka FR3 + Sharpa + 固定头部相机

适用流程：**示教手动拖动 → 松开并停稳 → 采集 ≥10 个姿态 → 求解 → 用新姿态独立验证**。
浏览器固定为硬件只读；启动服务器、新建会话都不会连接机器人，只有点击“连接预览”才会打开读取连接。
程序不启动示教、不发运动命令、不回零、不改变负载、不控制 Sharpa。

以下命令示例假定工程复制到 `/home/descfly/handeye`，原遥操工程在 `/home/descfly/llx/gello_upper_body_teleop`；只需按实际路径修改。
本手册的 Docker 路径针对原仓库当前的 **ROS 2 Humble 遥操镜像**，并不假定工控机宿主系统已经安装 ROS 或 libfranka。

## 1. 两种“末端位姿读取”是什么意思

标定需要同时知道两件事：图像中“相机看到的板的位姿”，以及该图像时刻“机械臂法兰在基座下的位姿”。
前者始终由相机图像、CameraInfo 内参和 ChArUco 板检测求出；界面里的两个选项只决定后者来自哪里。

| 比较项 | libfranka 只读 · 手动示教（本次优先使用） | 已有 ROS TF · 只读订阅 |
| --- | --- | --- |
| 法兰位姿从哪里来 | 本程序通过 libfranka 直接读取 FR3 实测状态 | 本程序监听其他 ROS 节点已经发布的 TF 坐标链 |
| 数据路径 | FR3 → libfranka 读取器 → 标定程序 | FR3 状态驱动／状态发布器 → ROS `/tf`、`/tf_static` → 标定程序 |
| 本程序是否直接连接机器人 IP | 是，连接所选臂 | 否，只订阅 ROS |
| 是否依赖机械臂 ROS 驱动 | 不依赖其 TF；仍需 ROS 读取相机 | 需要有其他节点提供真实、持续更新的末端 TF |
| 本程序是否发运动命令 | 不发 | 不发；但其他 TF 发布节点是否控制机器人，要由现场核对 |
| 时间配对 | 图像时刻前后两个主机接收时间的状态插值，间隔≤50ms | 查询图像时间戳对应的历史 TF，不用最新 TF 替代 |
| 采样时的机器人模式检查 | 必须是 `idle`；`guiding`、`move`、故障等状态拒绝 | 不读取 Franka 的模式枚举，只检查位姿和视觉观测是否稳定 |
| 额外要求 | 编译 C++ 只读读取器，使用兼容的 libfranka，释放该臂已有 FCI 客户端 | TF 来自真实关节状态，示教时仍能更新；坐标名称、时钟及 ROS 域一致 |

**建议这次选 libfranka。** 它适合停止遥操控制后直接读取机器人的使用方式，避免为了取得 TF 再启动运动控制器。
若现场已经有一个不干扰示教的真实 TF 状态链，才选择 TF；仅打开 RViz 或发布静态末端变换不能代替真实位姿。

默认坐标是 `left_fr3_link0 → left_fr3_link8`，右臂同理。libfranka 的 `O_T_EE` 可能包含工具偏移，所以程序计算
`T_base_flange = O_T_EE @ inverse(F_T_EE)`，对应默认的法兰 link8，不把 Sharpa 指尖或任意 TCP 当成法兰。
TF 路径则依赖发布者的运动学模型和坐标约定；不能假定不同模型或不同参考坐标下的数值一定相同。

“只读”指本程序只获取数据，**不是说它会自动进入可拖动状态，也不是说它不需要建立网络连接**。
示教状态由人员在 Franka 原生界面／示教按钮上准备。不同机器人系统版本下的 FCI 连接与示教兼容性仍须现场确认。

## 2. 需要哪些依赖

| 层次 | 依赖 | 用途 |
| --- | --- | --- |
| 工控机 | 原遥操系统已使用的 Docker Engine、Docker Compose 插件及浏览器 | 复用相机／机器人部署环境 |
| 机器人 SDK | 原遥操镜像中与实际机器人版本匹配的 libfranka 开发文件和动态库 | 编译／运行只读状态读取器；TF 路径不使用它读取机器人 |
| ROS 2 | Humble、`rclpy`、`sensor_msgs`、`cv_bridge`、`tf2_ros`、`rmw_cyclonedds_cpp` | 接收相机、内参与可选 TF |
| Python | Python 3.10（Humble）、NumPy、SciPy、带 `aruco` 的 OpenCV | 检测标定板、PnP、手眼求解 |
| 编译工具 | CMake、C++17 编译器、libfmt 开发文件 | 编译 `franka_state_reader` |
| 测试，可选 | pytest | 离线回归，不连接硬件 |

浏览器服务使用 Python 标准库，无需 Node.js、npm、PyQt、GPU/CUDA，也无需在本工程另装 Sharpa SDK。
相机继续由已有 Orbbec ROS 驱动持有，本工程不会再打开一个相机 SDK 客户端。

新提供的 `docker/Dockerfile` 从现有 `franka-upper-body-teleop:latest` 派生一个独立的 `franka-handeye:local` 镜像，补齐依赖，保留原 libfranka。
其中 Python 数值／视觉库固定为 NumPy 1.26.4、SciPy 1.13.1、OpenCV-contrib 4.10.0.84，放在容器的独立 venv，避免影响原遥操 Python 环境。
不用在宿主机执行 `pip install -r requirements.txt`，也不要为标定把原系统的 libfranka 换成最新版。

## 3. 首次复制与安装（推荐 Docker 路径）

### 3.1 复制工程

复制源码、配置、`tools/`、`handeye_web/`、`native/`、`docker/`、`tests/`、本手册和启动脚本。
不必复制 `runs/`、`__pycache__/`、旧虚拟环境及 `native/build*`。特别是本机的 CMake 缓存和二进制不能作为工控机编译结果使用。
新的容器路径使用单独的 `native/build-docker`，不会使用本机 `native/build` 缓存。

在工控机终端设置这几个变量（以后新开终端也先执行）：

```bash
export HANDEYE_DIR=/home/descfly/handeye
export TELEOP_REPO=/home/descfly/llx/gello_upper_body_teleop
export HANDEYE_UID="$(id -u)"
export HANDEYE_GID="$(id -g)"
chmod +x "$HANDEYE_DIR/start_franka_handeye.sh"
```

路径必须存在，且当前用户能读写 `HANDEYE_DIR`。

### 3.2 确认已有部署环境

```bash
docker compose version
docker image inspect franka-upper-body-teleop:latest --format '{{.Id}}'
test -f "$TELEOP_REPO/docker/.env"
```

若镜像不存在，先按原遥操工程的部署说明准备已验证的镜像；本标定工程不替代 FR3 和 Orbbec 驱动的首次安装。
原仓库 Dockerfile 记录 libfranka 0.19.0，但现场镜像和机器人版本才是依据，不因这个数字重新安装 SDK。

原工程的 CycloneDDS 配置要求至少 8MiB 的系统 UDP 接收缓冲。检查：

```bash
sysctl net.core.rmem_max
```

若小于 `8388608`，在机器人 FCI／采集均已停止时，按原工程的方法进行一次性设置：

```bash
sudo bash "$TELEOP_REPO/ops/setup/install_dds_buffers.sh"
```

已有正常配置不重复修改。不要在运行中的 FCI 会话里重启网卡或改机器人／相机网段。

### 3.3 构建标定依赖镜像

```bash
docker build \
  -f "$HANDEYE_DIR/docker/Dockerfile" \
  -t franka-handeye:local \
  "$HANDEYE_DIR"
```

此步需要网络下载缺少的 apt／Python 包；不启动机器人、相机或标定服务，不修改原遥操镜像。
若现场已验证的遥操镜像使用其他标签，构建时加 `--build-arg TELEOP_IMAGE=实际镜像:标签`；该镜像需是上述 Humble 布局。

### 3.4 可选：先运行无硬件回归

```bash
cd "$TELEOP_REPO/docker"
docker compose --profile tools \
  -f compose.yaml \
  -f "$HANDEYE_DIR/docker/compose.handeye.yaml" \
  run --rm --no-deps tools -lc \
  'source /opt/ros/humble/setup.bash && /opt/handeye-venv/bin/python -m pytest -q tests'
```

这里的 ROS 输入测试发布的是合成图像／TF，只在测试进程使用的测试话题上运行，不读取真实机器人。
如果要与现场 ROS graph 完全隔离，可在 `run` 的 `tools` 前添加 `-e ROS_DOMAIN_ID=177`。

## 4. 每次开始前准备机械安装与示教

1. 停止当前被标定臂的 GELLO、策略部署、回放、关节阻抗控制等运动会话。选 libfranka 时，还要释放该臂已有 FCI 客户端；本程序不会自动停止它们。
2. 在 Franka Desk 按现场流程完成准备和解锁，使人员可通过原生示教按钮手动引导。读连接所需的 FCI 状态按当前机器人系统版本操作；本程序不替你启用 FCI 或示教。
3. 固定头部相机／头部姿态，两臂基座保持不动。
4. 将现有板牢固装到被标定臂的法兰／手部支架：14×9 格、20mm 格、15mm 标记、`DICT_5X5_250`、legacy=true。
5. 若 Sharpa 夹持标定板，使用现场已有方式保持抓握，不能开启会改变板安装关系的手套跟随或改变手指姿态。标定程序本身不维持或调整抓握。
6. 保证只看到当前使用的这一块相同 ID 标定板，避免另一臂同 ID 板同时入镜。

## 5. 启动相机

只需要现有相机驱动，无需启动遥操 Operator GUI 或 rosbag 录制。

单头部相机部署，在终端 A：

```bash
cd "$TELEOP_REPO/docker"
docker compose up -d orbbec
docker compose logs --tail=60 orbbec
```

这个服务由原仓库的单相机入口提供，预期话题为 `/camera/color/image_raw` 和 `/camera/color/camera_info`。
相机参考地址 `192.168.1.123:8090`，文档参考序列号 `CP4N651006F`；用实际日志核对，因为设备曾更换。

若已通过原三相机入口在发布 `/cam0/color/image_raw`，沿用原驱动，在浏览器选择 `/cam0`，不要再启动 `orbbec` 服务。
Orbbec Viewer、单相机服务与三相机驱动不能重复占用同一相机。

可在相机容器中检查内参：

```bash
docker compose exec orbbec bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic echo /camera/color/camera_info --once'
```

在画面出现但无法采样时，也要核对 Image 与 CameraInfo 的 optical frame、分辨率和内参。

## 6. 启动标定工作台

在终端 B 先执行第 3.1 节的四个 `export`，然后：

```bash
cd "$TELEOP_REPO/docker"
docker compose --profile tools \
  -f compose.yaml \
  -f "$HANDEYE_DIR/docker/compose.handeye.yaml" \
  run --rm --no-deps tools
```

覆盖文件只为本次 `tools` 容器选用标定镜像。它继承原 Compose 的 host 网络、CycloneDDS、ROS 域、localhost 发现范围和 CPU 分配。
额外挂载标定目录，按当前用户 UID/GID 写文件。启动时在容器里编译只读读取器，然后启动网页服务；不会启动其他 Compose 服务。

终端出现 `Franka hand-eye: http://127.0.0.1:8765` 后，在**工控机自己的浏览器**打开：

[http://127.0.0.1:8765](http://127.0.0.1:8765)

程序只监听 loopback；另一台电脑上的 `127.0.0.1` 不会自动指向工控机。
关闭浏览器不会停止终端中的标定服务；结束服务按终端 B 的 Ctrl+C。
默认样本保存在宿主机的 `$HANDEYE_DIR/runs/franka_handeye/`，不会随临时容器删除。

## 7. 浏览器设置与预览

1. 先选左臂，默认 IP `172.16.26.5`；右臂为 `172.16.26.4`。按实际网络核对。
2. 选择相机话题组：单头部相机选 `/camera`，已有三相机采集选 `/cam0`。
3. “末端位姿读取”选择 **libfranka 只读 · 手动示教**。
4. 高级 JSON 中核对 `robot_ip`、`base_frame`、`tool_frame`、两个相机话题和板尺寸。改参数后新建会话才会生效；左／右切换也需要新会话。
5. 点击“新建标定会话”，再点击“连接预览”。这时才读取所选臂和相机。
6. 检查画面中的标记边框、ChArUco 交点、板坐标轴，应该有有效的标记数与重投影误差。板清晰可见但检测不对时先核对尺寸／字典／ID，不继续采样。

如使用 ROS TF，改选“已有 ROS TF · 只读订阅”，其余相机和采样流程相同。
应事先用同一 ROS 环境检查：

```bash
ros2 run tf2_ros tf2_echo left_fr3_link0 left_fr3_link8
# 右臂改为 right_fr3_link0 right_fr3_link8
```

这是调试命令，Ctrl+C 结束。手动拖动时数值应变化；停下后应稳定，且实际数据确实来自所选机器人。
仅存在一帧 TF 不够，标定程序还必须能查询到**图像时间戳**的历史 TF。

## 8. 手动拖动并采样

每个姿态按以下顺序操作：

1. 使用 Franka 原生示教按钮手动拖动，不点击其他系统的 Start arm、Home 或轨迹执行。
2. 将板放到清晰可见的位置并改变适量倾角；避开反光、遮挡、图像边缘和过远的小目标。
3. 松开示教按钮和机械臂，等待至少 1 秒。
4. 默认 libfranka 路径要求机器人报告 `idle`，并同时满足视觉板和法兰稳定检查。若仍为 `guiding`、`move` 或故障，先处理机器人状态，不强行采样。
5. 等界面提示“已停稳，可以采样”，点击“采集当前姿态”。确认计数增加，姿态记录出现新行。
6. 再拖动到另一个姿态，重复以上步骤。所有样本和检查图即时落盘。

**最少 10 组即可求解，建议 25–30 组。** 姿态要同时覆盖不同位置／距离，以及至少两个不同旋转轴约 20–40°的变化。
近重复姿态（与已采样姿态相比，平移不足 2cm 且旋转不足 5°）会拒绝；只平移或只绕一根轴即使达到 10 组，也可能因运动退化无法求解。

libfranka 状态以主机接收时间与相机配对，并非硬件触发同步；相机应使用同一工控机 system/global 时间。
停稳采样是这个流程的必要部分，不应边拖边点采集。

## 9. 求解与质量检查

采足至少 10 个姿态，点击“求解标定”。程序先以每第 5 个姿态做留出检查，再对全部样本拟合最终矩阵。
10 个姿态时是 8 个拟合、2 个留出；留出检查后最终结果用全部 10 个姿态重拟合。

全样本拟合和留出检查都需要满足：

| 指标 | 门槛 |
| --- | --- |
| 每帧角点重投影 RMSE | ≤3px |
| 平移 RMSE | ≤10mm |
| 平移最大误差 | ≤10mm |
| 旋转 RMSE | ≤2° |
| 旋转最大误差 | ≤5° |

`calibration.json` 的 `quality_pass=true` 说明拟合和留出检查通过，还需要下一步独立验证。
未通过仍会保存诊断结果，不能把“生成了 JSON”当作“标定通过”。先检查板安装、时钟、内参、板尺寸和姿态覆盖。
继续追加样本会使旧结果移到 `.previous`，需要重新求解。

## 10. 独立验证

**不要拆板、改变抓握、移动相机或基座。** 在通过的标定会话中点击“新建独立验证会话”。
程序在新目录保存该标定结果的固定副本，不会把验证数据加入拟合。

1. 验证会话新建后，重新点击“连接预览”。
2. 手动拖动到没有用于拟合的新姿态，仍按“松开 → 停稳 → 采样”。
3. 至少 5 个新姿态，建议 10 个。
4. 点击“检验新姿态（不重拟合）”，检查相同的平移和旋转门槛。
5. `validation.json` 中的 `validation_pass=true` 才表示此次独立新姿态检查通过。

验证不通过时先排查原因；若板或相机安装已经变化，重新标定，不能依靠修改阈值通过。
通过后才拆板去另一臂；为另一臂新建标定会话，完整重复采样、求解和独立验证。两臂默认结果分别相对于各自 link0。

## 11. 结果文件及变换方向

```text
$HANDEYE_DIR/runs/franka_handeye/live/left/<标定会话>/
  samples.json
  sample_001.png ...
  calibration.json
$HANDEYE_DIR/runs/franka_handeye/live/left/<验证会话>/
  samples.json
  sample_001.png ...
  calibration_reference.json
  validation.json
```

右臂保存在 `live/right/`。Docker 中显示的 `/workspace/handeye/runs/...` 就是宿主机上述挂载路径。

```text
T_base_flange @ T_flange_board = T_base_camera @ T_camera_board
p_base = T_base_camera @ p_camera_optical
```

`T_base_camera` 将相机彩色 optical frame 中的点变换到对应机械臂基座；`T_camera_base` 是其逆变换，平移／点坐标单位为米。
兼容旧命名的 `T_base_cam0` 和 `T_cam0_base` 也保留。`T_tool_board` 是本次板相对法兰的安装偏移，不是 Sharpa 指尖 TCP。
应同时核对 `parent_frame`、`child_frame`、`tool_frame`，不能将左右臂外参或相机 optical frame 混用。

`deployment_eligible=false` 始终保留，表示本工具不会自动授权或安装到机器人；它不是拟合失败标志。
独立验证结果不会回写原拟合 JSON 的 `independent_validation_pass` 字段，应读取对应 `validation.json`。
标定残差也不等于实际 TCP 接触精度；用于任务前仍须核对工具偏移与实际任务误差。

## 12. 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 找不到原遥操镜像 | 工控机使用的实际镜像标签；先准备原系统已验证的部署镜像 |
| CMake 报缓存路径不同 | 不复制本机 `native/build*`；容器使用 `native/build-docker`，本机方案使用新的 `native/build-ipc` |
| 找不到 `FrankaConfig.cmake` 或 `libfranka.so` | SDK 是否只在容器里；编译环境和运行环境是否使用同一安装；优先用本手册 Docker 路径 |
| `IncompatibleVersionException` | libfranka 与机器人系统版本是否匹配，不盲目升级；复用已正常运行的原镜像 |
| 找不到 `cv2.aruco` | 是否装了带 contrib/aruco 的 OpenCV，是否误用了其他 Python |
| `cv_bridge` 或 NumPy ABI 报错 | 使用 ROS 对应 Python；不要在原环境直接升级 NumPy 2.x；派生镜像使用隔离的 NumPy 1.x |
| 画面／CameraInfo 一直等待 | 相机驱动是否启动、话题 `/camera` 或 `/cam0` 是否正确、DDS 的 ROS 域／RMW／localhost 范围是否一致 |
| 主机可 ping，但没有图像 | 相机网段、驱动连接日志、设备占用与图像话题；ping 不证明流和 FCI 可用 |
| UDP receive buffer／DDS 初始化失败 | 第3.2节的系统缓冲是否达标，配置路径在宿主机和容器中是否有效 |
| Franka 只读状态等待或连接退出 | 机器人地址、现场 FCI 准备、SDK 版本、网络及该臂其他客户端占用 |
| 一直提示 `guiding`／非 `idle` | 松开示教按钮；核对现场机器人状态，按现场流程处理；程序不会强制切换状态 |
| 等待覆盖图像时刻的状态 | 相机是否启用主机时间同步／system/global time，是否和读取器在同一台工控机，是否有延迟／中断 |
| TF 找不到或时间外推错误 | 动态 TF 发布者、坐标名、图像时刻历史缓存；不要用静态法兰或最新 TF 替代 |
| 姿态已采集／旋转变化不足 | 增加真实姿态多样性，尤其至少两个旋转轴 |
| 权限错误 | 当前用户能否写标定目录，UID/GID 是否设置正确，是否误复制了 root 所有的旧输出 |
| 结束后文件“丢失” | 看宿主机 `HANDEYE_DIR/runs`；不要只看临时容器文件系统 |

## 13. 已有宿主机 ROS／SDK 时的本机运行方案

若工控机宿主机已有可用的 ROS Python 环境及匹配 libfranka，也可不使用派生容器。
Ubuntu 22.04 常用 Humble/Python3.10，24.04 常用 Jazzy/Python3.12；尽量匹配数据源，不能把 Humble 的 Python 扩展直接装进 Python3.12。
若宿主机根本没有 ROS，请按官方发行版安装说明准备，或直接使用上述 Docker 路径，不要只安装 Python wheel 来代替 ROS。

示例为 Ubuntu24.04/Jazzy；Humble 环境将发行版变量改成 `humble`：

```bash
HAND_EYE_ROS_DISTRO=jazzy
sudo apt update
sudo apt install build-essential cmake libfmt-dev python3-venv \
  python3-opencv python3-numpy python3-scipy python3-pytest \
  "ros-${HAND_EYE_ROS_DISTRO}-rclpy" \
  "ros-${HAND_EYE_ROS_DISTRO}-sensor-msgs" \
  "ros-${HAND_EYE_ROS_DISTRO}-cv-bridge" \
  "ros-${HAND_EYE_ROS_DISTRO}-tf2-ros" \
  "ros-${HAND_EYE_ROS_DISTRO}-rmw-cyclonedds-cpp"
source "/opt/ros/$HAND_EYE_ROS_DISTRO/setup.bash"
python3 -c 'import cv2,numpy,scipy,rclpy,cv_bridge,tf2_ros; print(cv2.__version__, hasattr(cv2,"aruco"))'
```

如果 OpenCV／SciPy 不满足需求，用匹配当前 ROS Python 的独立环境，不修改原遥操环境：

```bash
cd "$HANDEYE_DIR"
python3 -m venv --system-site-packages .venv-ipc
.venv-ipc/bin/python -m pip install \
  numpy==1.26.4 scipy==1.13.1 opencv-contrib-python==4.10.0.84
export HANDEYE_PYTHON="$HANDEYE_DIR/.venv-ipc/bin/python"
```

编译读取器并指定路径：

```bash
cd "$HANDEYE_DIR"
cmake -S native -B native/build-ipc
cmake --build native/build-ipc -j2
export HANDEYE_FRANKA_READER="$HANDEYE_DIR/native/build-ipc/franka_state_reader"
```

找不到 SDK 时，可给配置命令加 `-DFranka_DIR=/实际安装目录/lib/cmake/Franka`；动态库也必须在运行时可找到。
若 SDK 只在原 Docker 镜像中，改走 Docker 路径，不拷贝容器的 `.so` 到宿主机拼接环境。

以原部署的 CycloneDDS 配置接入同机相机：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$TELEOP_REPO/config/cyclonedds.xml"
# ROS_DOMAIN_ID 不设时，启动器从原工程 docker/.env 读取 TELEOP_ROS_DOMAIN_ID。
# 若已有 ROS_DOMAIN_ID，确认其和相机实际使用的域一致。
cd "$HANDEYE_DIR"
./start_franka_handeye.sh
```

`TELEOP_REPO`、`HANDEYE_FRANKA_READER` 和可选的 `HANDEYE_PYTHON` 要在同一启动终端保留。
启动器不会启动相机或驱动，也不会替你设置以上 DDS 环境变量。
如果 ROS overlay 只为容器构建、路径或发行版不匹配，不在宿主机 source 它；选 Docker 路径。

## 14. 本次核验范围与来源

本机验证包括原环境及上述固定版本 NumPy/SciPy/OpenCV 的离线回归、真实 ROS 传输的合成图像／TF 测试、读取器本机编译，以及 Compose 覆盖后的合并配置。
本机没有 `franka-upper-body-teleop:latest` 基础镜像，未实际构建／运行派生镜像；尚未连接工控机、实际 FR3 或 Orbbec。
因此不能把本机软件检查当作现场 FCI／示教、Humble 容器运行或标定精度验收。

来源：

- 原工程 [Dockerfile](https://github.com/ma39nong/Franka_Gello_Sharpa_armhand_-_-/blob/main/docker/Dockerfile)、[Compose](https://github.com/ma39nong/Franka_Gello_Sharpa_armhand_-_-/blob/main/docker/compose.yaml)、[Orbbec配置说明](https://github.com/ma39nong/Franka_Gello_Sharpa_armhand_-_-/blob/main/docs/ORBBEC_CAMERA.md)。
- Franka 官方 [read/readOnce与机器人状态](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html)、[Robot API](https://frankarobotics.github.io/libfranka/latest/classfranka_1_1Robot.html)。
- Docker 官方 [多 Compose 文件合并](https://docs.docker.com/compose/how-tos/multiple-compose-files/merge/)。
