"""Training-only costs and exploration settings; no inference action override."""


def boundary_cost(world, key, cfg, margin=0.20, weight=0.3):
    r = world.robot(key)
    clearance = min(
        cfg.length / 2 - cfg.robot_radius - abs(r.x), cfg.width / 2 - cfg.robot_radius - abs(r.y)
    )
    return weight * max(0.0, (margin - clearance) / margin) ** 2


def training_settings(role, precision=False):
    if precision:
        if role == "manager":
            return {
                "learning_rate": 1e-4,
                "mse_limit": 0.02,
                "delta_limit": 2.0,
                "choice_log_std": -1.2,
                "target_log_std": -8.0,
                "gate_log_std": -1.5,
                "preserve_coordinate_function": True,
            }
        if role in ("defender", "keeper"):
            return {
                "learning_rate": 3e-6,
                "mse_limit": 3e-5,
                "delta_limit": 0.06,
                "motion_log_std": -3.5,
            }
    if role == "manager":
        # The Gaussian policy samples latent option/robot scores as well as targets.
        # Choice variance must be large enough to cross an argmax boundary.
        return {
            "learning_rate": 1e-5,
            "mse_limit": 5e-4,
            "delta_limit": 0.2,
            "choice_log_std": -0.5,
            "target_log_std": -4.5,
        }
    if role in ("defender", "keeper"):
        return {
            "learning_rate": 1e-5,
            "mse_limit": 3e-4,
            "delta_limit": 0.15,
            "motion_log_std": -3.5,
        }
    raise ValueError("Unknown learning role")
