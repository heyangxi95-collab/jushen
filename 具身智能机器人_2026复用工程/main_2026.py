#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026 年具身智能机器人比赛主流程框架。

本文件负责组织复用的 2025 年 ROS 模块。在 JSON 配置的现场参数完成标定，
且具备规则要求的识别证据适配器前，程序会拒绝启动正式比赛。
"""

import argparse
import json
import time
from pathlib import Path


class CompetitionController:
    """开始信号 -> 搜索与识别 -> 抓取 -> 送至得分区 -> 离场。"""

    def __init__(self, config_path):
        import rospy
        from std_msgs.msg import String
        from catch_ground.src.catch import KinovaRobot
        from catch_ground.src.realsense_yolo11 import RealSenseYolo11Detector
        from navigator2 import Navigator
        from summer_tts_speaker import SummerTTSSpeaker

        self.config_path = Path(config_path)
        self.config = self._load_config(self.config_path)
        self._validate_match_config()
        self.rospy = rospy
        if not rospy.core.is_initialized():
            rospy.init_node("embodied_2026_competition", anonymous=False)

        self.navigator = Navigator(self.config["waypoints"])
        self.speaker = SummerTTSSpeaker()
        self.kinova = KinovaRobot(self.config["arm"]["robot_type"])
        vision = self.config["vision"]
        self.detector = RealSenseYolo11Detector(
            weights=Path(vision["weights"]),
            imgsz=vision["image_size"],
            conf_thres=vision["confidence"],
        )
        self.running = False
        self.delivered_targets = []
        self._start_subscriber = rospy.Subscriber(
            self.config["competition"]["start_topic"], String, self._on_start, queue_size=1
        )
        self._record_event("ready")
        self.speaker.speak("机器人已就绪，等待比赛开始")

    @staticmethod
    def _load_config(config_path):
        with config_path.open("r", encoding="utf-8") as stream:
            return json.load(stream)

    def _validate_match_config(self):
        if not self.config.get("ready_for_match", False):
            raise ValueError("配置仍是模板。完成标定后才可将 ready_for_match 设为 true。")
        serialized = json.dumps(self.config, ensure_ascii=False)
        if "TODO" in serialized:
            raise ValueError("配置中仍有 TODO，禁止开始正式比赛。")
        weights = Path(self.config["vision"]["weights"])
        if not weights.is_file():
            raise FileNotFoundError("未找到本届目标检测权重：{}".format(weights))
        if not self.config["competition"].get("evidence_adapter_ready", False):
            raise ValueError("规则要求带框和时间戳的识别图片，先完成 evidence adapter。")

    def _on_start(self, message):
        keyword = self.config["competition"]["start_keyword"]
        if message.data.strip().casefold() != keyword.casefold():
            return
        if self.running:
            return
        self.control()

    def control(self):
        """正式比赛路径，替代 2025 年的客人服务任务序列。"""
        self.running = True
        competition = self.config["competition"]
        deadline = time.monotonic() + competition["time_limit_seconds"]
        self._record_event("started", deadline=deadline)
        self.speaker.speak("比赛开始，开始自主搜索")

        try:
            self.kinova.close_finger()
            if not self._navigate_with_deadline("entry", deadline):
                return
            self._search_grasp_and_deliver(deadline)
            if self.delivered_targets and self._remaining(deadline) > 0:
                self._autonomous_exit(deadline)
            elif not self.delivered_targets:
                self._record_event("exit_skipped", reason="no_successful_delivery")
        finally:
            self.navigator.stop()
            self.kinova.goto_home()
            self._record_event("stopped", delivered=self.delivered_targets)
            self.running = False

    def _search_grasp_and_deliver(self, deadline):
        """循环搜索点，并优先处理高价值等级目标。

        仅检测到目标不能计分。只有识别证据、抓取确认和得分区投放均通过各自的
        适配器确认后，目标才会进入下一阶段。
        """
        reserve = self.config["competition"]["exit_reserve_seconds"]
        pending = list(self.config["targets"])
        pending.sort(key=lambda target: target["level"], reverse=True)

        for waypoint_id in self.config["search_order"]:
            if self._remaining(deadline) <= reserve or not pending:
                break
            if not self._navigate_with_deadline(waypoint_id, deadline):
                continue
            candidates = [target for target in pending if target["search_waypoint"] == waypoint_id]
            for target in candidates:
                if self._remaining(deadline) <= reserve:
                    return
                detection = self._detect_target(target)
                if detection is None:
                    self._record_event("not_detected", target=target["id"])
                    continue
                evidence = self._save_recognition_evidence(target, detection)
                if not evidence:
                    self._record_event("recognition_not_scoreable", target=target["id"])
                    continue
                self.speaker.speak("识别到{}".format(target["display_name"]))
                self._record_event("recognized", target=target["id"], evidence=evidence)
                if not self._grasp_target(target, detection, deadline):
                    self._record_event("grasp_failed", target=target["id"])
                    continue
                if self._deliver_to_score_zone(target, deadline):
                    pending.remove(target)
                    self.delivered_targets.append(target["id"])
                    self.speaker.speak("{}已送达得分区".format(target["display_name"]))

    def _detect_target(self, target):
        self.kinova.goto_detect()
        return self.detector.detect_targets(
            target_items=[target["model_label"]],
            max_retry=self.config["vision"]["max_retry"],
            show_window=False,
        )

    def _save_recognition_evidence(self, target, detection):
        """规则要求的原图、目标框、名称和时间戳的集成接口。

        迁移的 2025 年检测器会返回类别和三维点，却不会保留带标注的图像。应先在
        realsense_yolo11.py 中补充该适配器，再将 evidence_adapter_ready 设为 true。
        返回 None 是为了避免不符合规则的识别被计分。
        """
        self._record_event(
            "evidence_adapter_required",
            target=target["id"],
            label=detection.name,
            camera_point=[detection.x, detection.y, detection.z],
        )
        return None

    def _grasp_target(self, target, detection, deadline):
        """复用已验证的 2025 年地面抓取，其他抓取类型保留接口。

        旧的桌面抓取流程内置 2025 年权重和位姿，必须针对本场地重新标定；柜体
        目标还必须先实现开门或开抽屉动作序列，才能用于正式比赛。
        """
        profile = target["pick_profile"]
        if profile == "floor":
            self.kinova.catch_ground(result=detection)
            return self._verify_grasp(target)
        if profile == "tabletop_or_shelf":
            self._record_event("table_grasp_calibration_required", target=target["id"])
            return False
        if profile == "container":
            self._record_event("container_grasp_sequence_required", target=target["id"])
            return False
        raise ValueError("Unknown pick_profile: {}".format(profile))

    def _verify_grasp(self, target):
        """夹爪反馈或抓取后视觉确认的接口。"""
        self._record_event("grasp_verification_required", target=target["id"])
        return False

    def _deliver_to_score_zone(self, target, deadline):
        if not self._navigate_with_deadline("score_zone", deadline):
            return False
        self.kinova.open_finger()
        # 在此补充经标定的得分区投放姿态，以及相机或力反馈确认。
        self._record_event("delivery_verification_required", target=target["id"])
        return False

    def _autonomous_exit(self, deadline):
        if self._navigate_with_deadline("exit", deadline):
            self._record_event("autonomous_exit_complete")
            self.speaker.speak("任务完成，正在自主离场")

    def _navigate_with_deadline(self, waypoint_id, deadline):
        """使用迁移的 Navigator2 客户端，并增加明确的比赛超时限制。"""
        remaining = self._remaining(deadline)
        if remaining <= 0:
            return False
        waypoint = self.config["waypoints"][waypoint_id]
        goal = self.navigator.set_goal("map", waypoint["position"], waypoint["orientation"])
        self.navigator.clear_costmap_client()
        self.navigator.client.send_goal(goal)
        timeout = min(remaining, self.config["navigation"]["goal_timeout_seconds"])
        complete = self.navigator.client.wait_for_result(self.rospy.Duration(timeout))
        success = complete and self.navigator.client.get_state() == 3
        if not success:
            self.navigator.stop()
            self._record_event("navigation_blocked", waypoint=waypoint_id)
        return success

    def _remaining(self, deadline):
        return max(0.0, deadline - time.monotonic())

    def _record_event(self, event, **details):
        record = {"time": round(time.monotonic(), 3), "event": event}
        record.update(details)
        event_path = Path(self.config["competition"]["event_log"])
        event_path.parent.mkdir(parents=True, exist_ok=True)
        with event_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description="2026 embodied-intelligence competition controller")
    parser.add_argument("--config", default="config/arena_2026.example.json")
    args = parser.parse_args()
    CompetitionController(args.config)

    import rospy

    rospy.spin()


if __name__ == "__main__":
    main()
