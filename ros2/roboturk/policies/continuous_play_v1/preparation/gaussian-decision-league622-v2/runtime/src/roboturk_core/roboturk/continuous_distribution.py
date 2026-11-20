"""Categorical behavior exploration with deterministic, frozen neural motor heads.

The 35-output deployment contract stays unchanged. PPO's likelihood ratio is the
probability of the selected behavior, not the density of redundant Gaussian
scores and unused motor outputs. The raw neural means remain available for the
existing measured trust region. This class contains no tactical decision rules.
"""

import torch
from stable_baselines3.common.distributions import DiagGaussianDistribution
from stable_baselines3.common.policies import ActorCriticPolicy


class DecisionDistribution(DiagGaussianDistribution):
    logit_scale = 3.0

    def proba_distribution(self, mean_actions, log_std):
        super().proba_distribution(mean_actions, log_std)
        self.choices = torch.distributions.Categorical(
            logits=mean_actions[:, :7] * self.logit_scale
        )
        return self

    def sample(self):
        actions = self.distribution.mean.clone()
        choices = self.choices.sample()
        actions[:, :7] = -0.8
        actions[torch.arange(len(actions), device=actions.device), choices] = 0.8
        return actions

    def log_prob(self, actions):
        return self.choices.log_prob(actions[:, :7].argmax(1))

    def entropy(self):
        return self.choices.entropy()


class ContinuousDecisionPolicy(ActorCriticPolicy):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if self.action_space.shape != (35,):
            raise ValueError("Continuous decisions require 35 outputs")
        self.action_dist = DecisionDistribution(35)
