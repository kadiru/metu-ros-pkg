"""Explicit micro-behavior contracts for future play/strategy selection.

Listing a checkpoint path is not a qualification claim. Consumers must inspect
its manifest and evidence before enabling it in their scenario distribution.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class MicroBehaviorSpec:
    name: str
    entry: str
    outcome: str
    scripted_context: str
    learned_outputs: tuple[str, ...]
    checkpoint: str
    evidence: str
    inspiration: str | None = None


CMDRAGONS = "https://ssl.robocup.org/wp-content/uploads/2019/01/2015_ETDP_CMDragons.pdf"
TIGERS = "https://ssl.robocup.org/wp-content/uploads/2025/04/2025_ETDP_TIGERs-Mannheim.pdf"
ER_FORCE = "https://ssl.robocup.org/wp-content/uploads/2024/04/2024_ETDP_ER-Force.pdf"

CATALOG = {
    spec.name: spec
    for spec in (
        MicroBehaviorSpec(
            "mobile_intercept",
            "live two-attacker play; field-wide planar motion",
            "physical interception without robot collision or field exit",
            "threat projections, actuator limits and dribbler enable; no anchor-line clamp",
            ("attack_frame_vx", "attack_frame_vy", "omega"),
            "policies/self_play_v1/final/defender/policy.zip",
            "policies/self_play_v1/qualification-defense-audit.json",
            CMDRAGONS,
        ),
        MicroBehaviorSpec(
            "mobile_goalkeeper",
            "live holder or incoming shot; field-wide planar motion",
            "physical save without robot collision or field exit",
            "goal/threat geometry, actuator limits, latency and dribbler enable",
            ("attack_frame_vx", "attack_frame_vy", "omega"),
            "policies/self_play_v1/final/keeper/policy.zip",
            "policies/self_play_v1/qualification-defense-audit.json",
            ER_FORCE,
        ),
        MicroBehaviorSpec(
            "mobile_defense_transfer",
            "passer controls ball; teammate and two mobile opponents assigned",
            "one applied pass, >=1m travel, 0.4s stable reception",
            "receiving-target flank switches, roles and fixed2.5m/s pass",
            (
                "passer_vx",
                "passer_vy",
                "passer_omega",
                "pass_fire",
                "receiver_vx",
                "receiver_vy",
                "receiver_omega",
            ),
            "policies/self_play_v1/final/transfer/policy.zip",
            "policies/self_play_v1/qualification-final-audit.json",
            TIGERS,
        ),
        MicroBehaviorSpec(
            "mobile_pressure_finish",
            "stable receiver possession with both mobile opponents active",
            "retained carry, one applied shot and whole-ball goal",
            "parking point, observed-keeper corner selection, handoff and fixed shot speed",
            ("body_vx", "body_vy", "omega", "shot_fire"),
            "policies/self_play_v1/final/finish/policy.zip",
            "policies/self_play_v1/qualification-final-audit.json",
            ER_FORCE,
        ),
        MicroBehaviorSpec(
            "continuous_lane_guard",
            "assigned midfield line; live opponent play",
            "physical interception; continuous activity through attacker handoff",
            "role anchor, geometric threat projections, actuator limits and dribbler enable",
            ("attack_frame_vx", "attack_frame_vy", "omega"),
            "policies/learned_opponents_v1/defender/policy.zip",
            "policies/learned_opponents_v1/defender/qualification.json",
            CMDRAGONS,
        ),
        MicroBehaviorSpec(
            "continuous_goalkeeper",
            "assigned goal; live holder or incoming shot",
            "physical save without a goal in the declared challenge interval",
            "goal anchor, geometric threat projections, actuator limits, latency and dribbler enable",
            ("attack_frame_vx", "attack_frame_vy", "omega"),
            "policies/learned_opponents_v1/keeper/policy.zip",
            "policies/learned_opponents_v1/keeper/qualification.json",
            ER_FORCE,
        ),
        MicroBehaviorSpec(
            "pass_against_learned_defense",
            "passer controls ball; teammate and learned opponents assigned",
            "actual pass, >=1m travel, 0.4s stable reception",
            "receiving point and roles; fixed pass speed",
            (
                "passer_vx",
                "passer_vy",
                "passer_omega",
                "pass_fire",
                "receiver_vx",
                "receiver_vy",
                "receiver_omega",
            ),
            "policies/learned_opponents_play_v1/transfer/policy.zip",
            "policies/learned_opponents_play_v1/qualification.json",
            TIGERS,
        ),
        MicroBehaviorSpec(
            "open_lane_finish",
            "stable receiver possession; frozen learned keeper active",
            "retained carry, one applied shot and a whole-ball goal",
            "centered parking point, observed-keeper corner selection and handoff; fixed shot speed",
            ("body_vx", "body_vy", "omega", "shot_fire"),
            "policies/learned_opponents_play_v1/finish/policy.zip",
            "policies/learned_opponents_play_v1/qualification.json",
            ER_FORCE,
        ),
        MicroBehaviorSpec(
            "pass_around_defender",
            "passer controls ball; teammate assigned",
            "actual pass, >=1m travel, 0.4s stable reception",
            "receiving point and roles; fixed pass speed",
            (
                "passer_vx",
                "passer_vy",
                "passer_omega",
                "pass_fire",
                "receiver_vx",
                "receiver_vy",
                "receiver_omega",
            ),
            "policies/defended_pass_receive_v3/policy.zip",
            "policies/defended_pass_receive_v3/qualification.json",
            TIGERS,
        ),
        MicroBehaviorSpec(
            "draw_keeper_finish",
            "stable receiver possession after pass",
            "retained carry, actual shot, whole-ball goal without opponent touch",
            "latched draw/finish phase and aim target; fixed shot speed",
            ("body_vx", "body_vy", "omega", "shot_fire"),
            "policies/keeper_finish_v1/policy.zip",
            "policies/keeper_finish_v1/qualification.json",
            ER_FORCE,
        ),
        MicroBehaviorSpec(
            "block_shot",
            "reachable rolling shot; assigned defensive line and heading",
            "physical interception and 0.5s stable possession; no goal conceded",
            "predicted crossing point, contact-triggered recovery target; fixed heading",
            ("attack_frame_vx", "attack_frame_vy"),
            "policies/block_shot_v1/policy.zip",
            "policies/block_shot_v1/qualification.json",
            CMDRAGONS,
        ),
        MicroBehaviorSpec(
            "cover_pass_lane",
            "assigned ball-to-opponent lane",
            "within 0.08m of lane midpoint continuously for 3s; no collision",
            "threat assignment, midpoint and target velocity",
            ("attack_frame_vx", "attack_frame_vy"),
            "policies/cover_pass_lane_v1/policy.zip",
            "policies/cover_pass_lane_v1/qualification.json",
            CMDRAGONS,
        ),
    )
}
