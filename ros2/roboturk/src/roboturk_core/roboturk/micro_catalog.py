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
