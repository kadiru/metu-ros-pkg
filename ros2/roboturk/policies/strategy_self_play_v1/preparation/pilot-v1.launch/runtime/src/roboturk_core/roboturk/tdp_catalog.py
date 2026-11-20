"""New option contracts, without changing any earlier frozen behavior catalog."""

from .micro_catalog import CATALOG as PREVIOUS, MicroBehaviorSpec

TIGERS = "https://ssl.robocup.org/wp-content/uploads/2025/04/2025_ETDP_TIGERs-Mannheim.pdf"
SKUBA = "https://ssl.robocup.org/wp-content/uploads/2019/01/2013_ETDP_Skuba.pdf"

CATALOG = {**PREVIOUS}
for kind, entry, outcome, source in (
    (
        "receive",
        "announced reachable teammate pass",
        "actual kick then0.5s stopped stable reception",
        TIGERS,
    ),
    (
        "redirect",
        "reachable incoming pass and open outgoing goal lane",
        "one mouth kick within0.15s of first contact then whole-ball goal",
        TIGERS,
    ),
    (
        "clear",
        "loose own-third ball and assigned flank outlet",
        "0.4s control then one kick through outlet gate without opponent touch",
        SKUBA,
    ),
):
    CATALOG["tdp_" + kind] = MicroBehaviorSpec(
        "tdp_" + kind,
        entry,
        outcome,
        "scripted intent, eligibility/role commitment, actuator limits and fixed kick power",
        ("body_vx", "body_vy", "omega", "release"),
        f"policies/tdp_skills_v1/{kind}/policy.zip",
        f"policies/tdp_skills_v1/{kind}/qualification-audit.json",
        source,
    )
