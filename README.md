# wheel_leg_esp32

基于 **ESP32-S3 + Arduino(PlatformIO)** 的轮腿机器人控制工程。项目将姿态感知、腿部关节力矩控制、轮毂电机平衡控制以及 UDP 遥控整合在同一套固件中。

## 1. 项目简介

该工程面向双轮+双腿（四关节）机器人，核心包含：

- **IMU 姿态线程**：独立 FreeRTOS 任务持续读取 MPU6050 数据。
- **关节电机控制（MIT 协议 over CAN/TWAI）**：发送关节扭矩指令并读取关节状态。
- **轮毂电机控制**：结合姿态角和腿长状态进行 LQR 风格控制，输出轮子扭矩。
- **UDP 遥控接口**：通过 Wi-Fi 接收上位机数据包，更新转向、前后速度和目标高度等控制量。

## 2. 硬件/软件依赖

### 硬件

- ESP32-S3-DevKitC-1（`esp32-s3-devkitc-1`）
- MPU6050（I2C）
- 4 个关节电机（MIT 协议）
- 2 个轮毂电机（通过 `SF_Motor` 静态库驱动）
- CAN 总线收发器、供电与整机机械结构

### 软件

- [PlatformIO](https://platformio.org/)（推荐 VS Code 插件）
- 框架：Arduino
- 平台：espressif32

当前工程环境定义见 `platformio.ini`。

## 3. 工程结构

```text
.
├── platformio.ini            # 平台、板卡、构建链接配置
├── src/
│   ├── main.cpp              # 主流程：初始化、主循环、IMU任务
│   ├── Control/              # 轮控与腿部 VMC 控制逻辑
│   ├── Kinematics/           # 正逆运动学与雅可比相关计算
│   ├── MIT/                  # MIT 协议及 TWAI(CAN) 封装
│   ├── MPU6050/              # 传感器读取封装
│   ├── Remote/               # UDP 遥控通信封装
│   └── SF_Motor/             # 轮毂电机静态库与头文件
├── include/                  # 预留头文件目录
├── lib/                      # 预留私有库目录
└── test/                     # PlatformIO 测试目录
```

## 4. 控制流程概览

`setup()` 阶段：

1. 初始化 I2C、串口与 MPU6050
2. 创建 IMU 任务（固定到 Core1）
3. 初始化 UDP 通信
4. 初始化 CAN，并使能 4 个关节电机
5. 初始化轮毂电机

`loop()` 阶段：

1. 处理 UDP 收包
2. 获取轮速相关指令（当前以浮点流方式读取）
3. 接收关节 CAN 状态并执行关节控制
4. 执行轮子控制（姿态 + 速度/位移状态）
5. 由关节编码器更新关节角
6. 计算左右腿 VMC 输出
7. 下发左右轮毂电机目标力矩

## 5. 快速开始

### 5.1 克隆工程

```bash
git clone <your-repo-url>
cd wheel_leg_esp32
```

### 5.2 修改本地网络参数

在 `src/main.cpp` 中修改 Wi-Fi 配置：

```cpp
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
```

### 5.3 编译

```bash
pio run
```

### 5.4 烧录

```bash
pio run -t upload
```

### 5.5 串口监视

```bash
pio device monitor -b 115200
```

## 6. UDP 协议说明（当前实现）

接收端在 `RemoteUDP::parsePacket()` 中按帧解析：

- 帧头：`0xFF`
- ID：`0x01 / 0x02 / 0x03`
- 数据区：
  - `0x01`：2 个 `float`
  - `0x02`：2 个 `float`
  - `0x03`：1 个 `float`
- 帧尾：`0xFE`

默认仅接受指定来源 IP 的数据包（见 `handleIncomingPackets()` 的 `allowedIP`）。

## 7. 关键可调参数

- `src/Control/wheel_ctrl.cpp`
  - `speed_limit`：轮毂扭矩限幅
  - `calc_K1~calc_K4`：随腿长变化的控制参数拟合函数
- `src/main.cpp`
  - `up_start`：轮毂使能倍率
- `src/Control/motor_ctrl.cpp`
  - VMC 相关增益与目标高度参数

## 8. 注意事项

1. 当前仓库 `src/main.cpp` 中存在明文 Wi-Fi 账号密码，建议改为本地配置文件或编译宏注入，避免泄露。
2. `src/SF_Motor/libSF_Motor.a` 为预编译静态库，如更换芯片/SDK 版本可能需要重新编译该库。
3. 关节电机 ID、CAN 总线拓扑、电机零点与机械安装方向需与实际硬件一致。
4. 上电调试建议先将扭矩限幅设置保守值，并在支架状态下验证。

## 9. 后续建议

- 增加参数管理层（如 JSON/NVS）实现在线调参与保存。
- 为 UDP 命令加入 CRC 与版本字段，提高通信鲁棒性。
- 在 `test/` 增加运动学与协议解析的单元测试。

---

如需我继续完善：可以再补一版 **英文 README**、**接线图模板** 或 **上位机通信示例（Python）**。
