# CyberArm 数据集输出 v1

这是项目自定义的通用时序数据集结构，不宣称兼容 ROS bag、LeRobot、RLDS 或某个特定训练平台。CSV 遵循 RFC 4180 的表头、逗号分隔、CRLF 行尾与双引号转义约定，使用 UTF-8 无 BOM；UTC 日期时间采用 RFC 3339 格式。

- CSV 格式说明：https://www.rfc-editor.org/rfc/rfc4180
- 时间戳说明：https://www.rfc-editor.org/rfc/rfc3339

## 使用

数字孪生：连接双机后，在中间栏“数据集”点击“开始记录”，完成实验后点击“结束记录”，等待整理完成再点击“下载 ZIP”。经典版在“数据集输出”页操作。开始记录不会使能或运动电机，结束记录不会停止电机，实验动作仍需单独手动执行。

原始数据保存在程序目录的 `datasets/<session_id>/`，ZIP 在同级。每次记录创建随机唯一编号，不覆盖旧会话，不自动删除历史数据，也不将实验数据提交到 Git。重启后原始目录仍在；界面的最近一次记录状态仅在当前进程中维护，旧会话可从本地目录读取。

断开连接时结束本次记录。正常退出后台会先按原有逻辑停止/断开电机，再等待记录器整理。关闭网页不会结束后台记录，异常杀进程或断电可能留下 `complete=false` 的未完成会话。磁盘文件最多间隔约 1 秒刷新，不承诺断电不丢数据。

## 数据包

| 文件 | 内容 |
|---|---|
| `feedback.csv` | 每条有效 Type 2 反馈一行，每行一个关节；包括电机原始位置、软件坐标角度、速度、力矩反馈、温度、状态、零点/方向、目标/轨迹参考，以及原始 CAN ID 和负载 |
| `commands.csv` | 每次周期控制指令成功发送后的量化字段、增益、前馈、补偿分项、PD/总力矩估算、反馈年龄和原始报文；不是电机执行确认 |
| `parameters.csv` | 每次通过格式/地址/状态/请求相关性验证的参数读回单独一行；包括模式、力矩上限、电流上限、Iq 和母线电压 |
| `events.csv` | 应用事件；录制期间成功保存的配置通过 `configuration_json` 列附带完整快照 |
| `metadata.json` | 会话、起止时间、源代码 SHA-256、开始配置、各表行数、各关节反馈数、质量标记、结束原因、完整性状态与文件校验值 |
| `schema.json` | 每一列的名称、类型、单位、含义和空值约定 |
| `README.txt` | 随数据包携带的使用说明 |

`schema_version` 固定为 `cyberarm.dataset.v1`。字段顺序固定，所有数据表都带表头。

## 每行公共字段

| 字段 | 含义 |
|---|---|
| `session_id` | 会话唯一编号 |
| `sample_index` | 四个流共用的入队顺序号，不是电机帧计数 |
| `timestamp_utc` | UTC 日期时间，例如 `2026-10-14T08:00:00.123456789Z` |
| `timestamp_unix_ns` | 电脑系统时钟的 Unix 纳秒整数 |
| `monotonic_ns` | 电脑单调时钟的纳秒整数 |
| `elapsed_s` | 相对开始记录时刻的单调时间，单位秒 |

时间戳在主机处理反馈/参数或成功发送周期命令时取得，不是电机硬件采样时间，也不是 CAN 总线上到达的精确时间；串口与接收队列延迟未测量。纳秒位数代表时钟表示格式，不代表纳秒测量精度。跨线程时间戳不保证按入队顺序严格递增；分析时可按 `monotonic_ns`、`sample_index` 排序。

两个电机异步反馈，参数也是异步低频读回，不做硬件同步、插值、前向填充或伪造固定 100 Hz。样本数量取决于实际有效反馈，不能把网页 10 Hz 刷新频率当成实机采样率。计算持续时间优先使用 `elapsed_s` 或单调时钟，避免系统时间回拨的影响。

## 单位、坐标与缺失值

- 主分析角度使用 rad、速度使用 rad/s、力矩使用 N·m，另附关节角度 deg 方便查看；温度为 °C。
- 电机原始坐标与软件关节坐标明确分列：`position_joint_rad = direction * (position_motor_rad - software_zero_rad)`，软件速度乘相同方向系数。
- `torque_feedback_motor_nm` 是电机协议报告的电机坐标力矩，不是独立测力传感器，也不能直接当作人体施力。
- `target_*`、`setpoint_*` 是软件参考，`*_estimate_*` 是软件估算，`feedforward_*` 是已发送的命令字段，均不冒充实测。
- 未知值为空单元格，不用 0、旧值或字符串 NaN 代替。没有零点时仍可保留原始电机位置，软件角度为空。
- 布尔值为 `true` / `false`；字符串按 CSV 规则转义。以公式触发字符开头的文本另加前导单引号，避免表格软件把日志当公式执行。
- Unix 纳秒超出 JavaScript/Excel 浮点数的精确整数范围，导入时须作为 int64 或字符串。元数据中的起始纳秒以字符串保存。

Python 分析示例（pandas 是分析环境依赖，不是上位机运行依赖）：

```python
import pandas as pd
feedback = pd.read_csv(
    "feedback.csv",
    dtype={"timestamp_unix_ns": "Int64", "monotonic_ns": "Int64", "observed_uid": "string"},
)
shoulder = feedback[feedback["joint"] == "shoulder"].sort_values("monotonic_ns")
```

如需合并参数与反馈，应按关节、时间和最大允许年龄显式对齐，保留参数时间与年龄；不可把一次 Iq 读回复制成每一帧都有同步电流测量。

## 数据质量与校验

`complete=true` 只表示写盘器正常整理完成，不表示双机关节同步、实验达到目标或数据适合人体分析。`quality_flags` 标出无反馈、缺少某关节、身份未验证或故障反馈等情况。队列溢出或写盘失败会结束数据采集并标记不完整，不改变电机控制；请关注界面的记录异常提示。它只能统计主机记录器丢行，不能检测没有电机序号的 CAN 总线丢帧。

源文件校验值用于标识软件版本；数据文件 SHA-256 用于检测修改，不是数字签名，也不是防篡改认证。独立校验工具支持本地目录或 ZIP：

```text
python tools/validate_dataset.py datasets/<session_id>
python tools/validate_dataset.py datasets/<session_id>.zip
```

返回码：0 表示格式/字段/时间/行数/校验值通过且记录完整；1 表示结构或校验错误；2 表示结构可读但记录不完整。质量标记仍需结合实验目的判断。

隔离 UI 样例的 `data_origin=synthetic_test_only`，明确不是实机数据，不应用于验证电机出力或模型精度。
