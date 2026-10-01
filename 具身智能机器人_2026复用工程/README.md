# 2026 具身智能机器人复用工程

本工程只迁移了仍能服务 2026 任务的 2025 功能模块，并保持原有的相对目录和导入关系。迁移文件的函数逻辑、函数签名和导入关系均保持原样，仅将注释和文档字符串统一为中文；只有 `main_2026.py` 和 `config/` 是为今年规则新增的编排层。

## 规则对比与取舍

| 项目 | 2025 通用服务机器人 | 2026 具身智能机器人 | 本工程处理 |
| --- | --- | --- | --- |
| 任务 | 为客人取物、送到客人手中、清理垃圾 | 两台机器人无人同场，搜寻 8 个一级、6 个二级、4 个三级物品并放回各自得分区 | 移除客人、点单、垃圾流程；保留感知、导航、抓取、语音 |
| 场地 | 客人会移动 | 赛前允许建图，正赛使用地图；对手是动态障碍物 | 保留 `move_base` 导航；在主流程中对导航设置超时 |
| 识别得分 | 服务任务的一部分 | 每次识别需语音播报；争议时须提供有目标框、名称和时间戳的图片 | 保留 RealSense+YOLO；明确要求补充 evidence adapter 后才可正式运行 |
| 抓取与交付 | 家具/地面取物，交给客人或投入垃圾桶 | 一、二、三级物品抓取，送入己方 1m x 1m 得分区 | 保留 Kinova 低层控制和地面抓取；桌面/货架、柜门/抽屉、得分区姿态需要本场标定 |
| 时间与离场 | 完成两项服务任务后离场 | 10 分钟；至少成功识别、抓取并投放一件才可自主离场 | 主流程保留倒计时、预留离场时间和投放门槛 |

## 迁移文件

| 文件 | 原功能 | 2026 用途 |
| --- | --- | --- |
| `base_controller.py` | `/cmd_vel` 与底盘姿态 | 紧急停动、局部转向 |
| `navigator.py`、`navigator2.py` | `move_base` 目标导航与代价地图清理 | 房间巡航、回得分区、离场；主流程使用 `navigator2.py` |
| `summer_tts_speaker.py` | SummerTTS ROS 发布器 | 识别与阶段语音播报 |
| `soundplayer.py` | ROS sound_play 封装 | `navigator.py` 的原有依赖 |
| `camera_to_map.py`、`goal_calculator.py` | 视觉坐标转地图、生成面向目标的靠近点 | 需要精确靠近物品时的辅助工具 |
| `catch_ground/src/catch.py` | Kinova 笛卡尔控制、夹爪、地面和桌面抓取 | 复用机械臂低层控制和已验证的地面抓取路径 |
| `catch_ground/src/realsense_yolo11.py` | RealSense + YOLO11 + 深度三维坐标 | 本届目标检测入口 |
| `catch_ground/src/realsense_yolo11_desk.py` | 桌面近距离 RealSense 检测 | 桌面/货架抓取标定参考 |
| `catch_ground/src/realsense_camrea.py` | 带时间戳 RealSense 原图保存 | 识别证据适配的基础 |

未迁移的人脸识别、人体识别、语音点单、客人跟随、垃圾清理，以及 2025 的 `.pt` 权重均与本届任务不匹配。`models/` 中应放入按本届赛前清单训练的权重。

## 去年主流程

去年的最终入口是 `tongyong25.py` 的 `Controller`：初始化 ROS 和设备后，先入场；接着在多个点位寻找主人与客人，语音获取物品名称；再巡航找物、抓取、返回客人并放手；最后搜索垃圾、抓取、投入垃圾桶并离场。入口代码同时订阅 `/start_signal`，但构造函数又直接调用 `control()`，因此实际没有严格等待裁判开始信号。

今年的 `CompetitionController.control()` 将此改成：裁判开始信号 -> 入场 -> 按价值从高到低循环房间搜索 -> 识别播报和证据 -> 确认抓取 -> 回己方得分区并确认投放 -> 至少一件成功后自主离场。它不再包含人与客人相关分支。

## 使用前必须完成

1. 在 `config/arena_2026.example.json` 填入本场所有位姿、搜索点、18 个目标的最终类别和模型标签；把 `TODO` 全部清除。
2. 训练并配置本届 YOLO 权重，重新标定 RealSense/Kinova 外参、三类抓取姿态、得分区投放姿态。
3. 为 `realsense_yolo11.py` 增加“原图 + 目标框 + 名称 + 时间戳”的证据输出，并将 `evidence_adapter_ready` 置为 `true`。
4. 对桌面/货架抓取替换 `catch.py` 中硬编码的 2025 权重、抓取位姿；为三级物品实现柜门/抽屉开启与安全收回动作。
5. 将 `ready_for_match` 改为 `true` 后，再在 ROS Noetic 实机环境中执行：

```bash
python3 main_2026.py --config config/arena_2026.example.json
```

`main_2026.py` 会拒绝使用含 `TODO`、缺权重、未开启证据适配或尚未确认 `ready_for_match` 的配置开始比赛。

## 新写函数清单

所有新函数都在 `main_2026.py`，没有改写迁移文件：

- `CompetitionController.__init__`、`_load_config`、`_validate_match_config`：加载并拦截不安全/不完整配置。
- `_on_start`、`control`：接收开始信号并统筹完整比赛生命周期。
- `_search_grasp_and_deliver`、`_detect_target`：按规则循环搜索与调用旧的 RealSense 检测器。
- `_save_recognition_evidence`：证据输出接口，目前故意不通过计分，等待接入带框保存。
- `_grasp_target`、`_verify_grasp`、`_deliver_to_score_zone`：三类抓取、抓取确认、投放确认接口；只直接复用已验证的地面抓取。
- `_autonomous_exit`、`_navigate_with_deadline`、`_remaining`、`_record_event`：离场、带总时限导航、倒计时和事件记录。
- `main`：命令行入口。

这样做避免把去年针对客人、垃圾、旧权重和固定场地坐标的行为误用到今年的正式比赛中。
