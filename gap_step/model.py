from __future__ import annotations

import torch
from torch import nn
from torch.distributions import Normal

from gap_step.graph import EDGE_FEATURE_DIM, GLOBAL_FEATURE_DIM, NODE_FEATURE_DIM, GraphBatch


def _mlp(input_dim: int, hidden_dim: int, output_dim: int, layers: int = 2) -> nn.Sequential:
    modules: list[nn.Module] = []
    dim = input_dim
    for _ in range(max(1, layers - 1)):
        modules.extend([nn.Linear(dim, hidden_dim), nn.Tanh()])
        dim = hidden_dim
    modules.append(nn.Linear(dim, output_dim))
    return nn.Sequential(*modules)


class GNNLayer(nn.Module):
    def __init__(self, hidden_dim: int):
        super().__init__()
        self.message = _mlp(hidden_dim + hidden_dim, hidden_dim, hidden_dim)
        self.update = _mlp(hidden_dim + hidden_dim + hidden_dim, hidden_dim, hidden_dim)
        self.norm = nn.LayerNorm(hidden_dim)

    def forward(self, node_h: torch.Tensor, edge_h: torch.Tensor, edge_index: torch.Tensor, global_per_node: torch.Tensor) -> torch.Tensor:
        if edge_index.numel() == 0:
            agg = torch.zeros_like(node_h)
        else:
            src, dst = edge_index
            msg = self.message(torch.cat([node_h[src], edge_h], dim=-1))
            agg = torch.zeros_like(node_h)
            agg.index_add_(0, dst, msg)
            degree = torch.zeros((node_h.shape[0], 1), dtype=node_h.dtype, device=node_h.device)
            degree.index_add_(0, dst, torch.ones((dst.shape[0], 1), dtype=node_h.dtype, device=node_h.device))
            agg = agg / degree.clamp_min(1.0)
        delta = self.update(torch.cat([node_h, agg, global_per_node], dim=-1))
        return self.norm(node_h + delta)


class GNNTeacherActorCritic(nn.Module):
    def __init__(
        self,
        global_dim: int = GLOBAL_FEATURE_DIM,
        node_dim: int = NODE_FEATURE_DIM,
        edge_dim: int = EDGE_FEATURE_DIM,
        action_dim: int = 2,
        max_acc: float = 3.0,
        hidden_dim: int = 128,
        gnn_layers: int = 4,
        min_log_std: float = -0.5,
        max_log_std: float = 2.0,
        log_std_init: float = 0.0,
    ):
        super().__init__()
        self.global_dim = int(global_dim)
        self.node_dim = int(node_dim)
        self.edge_dim = int(edge_dim)
        self.hidden_dim = int(hidden_dim)
        self.gnn_layers = int(gnn_layers)
        self.max_acc = float(max_acc)
        self.min_log_std = float(min_log_std)
        self.max_log_std = float(max_log_std)
        self.log_std_init = float(log_std_init)
        self.global_encoder = _mlp(self.global_dim, hidden_dim, hidden_dim)
        self.node_encoder = _mlp(self.node_dim, hidden_dim, hidden_dim)
        self.edge_encoder = _mlp(self.edge_dim, hidden_dim, hidden_dim)
        self.layers = nn.ModuleList([GNNLayer(hidden_dim) for _ in range(self.gnn_layers)])
        graph_dim = hidden_dim * 5
        self.actor = _mlp(graph_dim, hidden_dim, action_dim)
        self.global_actor = _mlp(self.global_dim, hidden_dim, action_dim)
        self.critic = _mlp(graph_dim, hidden_dim, 1)
        self.guidance_prior_gain = 1.2
        self.log_std = nn.Parameter(torch.full((action_dim,), self.log_std_init))
        self._eps = 1e-6
        self._zero_actor_head()
        self._zero_head(self.global_actor)

    def encode_graph(self, batch: GraphBatch) -> torch.Tensor:
        global_h = self.global_encoder(batch.global_features)
        node_h = self.node_encoder(batch.node_features)
        edge_h = self.edge_encoder(batch.edge_features)
        global_per_node = global_h[batch.node_batch]
        for layer in self.layers:
            node_h = layer(node_h, edge_h, batch.edge_index, global_per_node)
        mean_pool = self._mean_pool(node_h, batch.node_batch, batch.num_graphs)
        max_pool = self._max_pool(node_h, batch.node_batch, batch.num_graphs)
        agent_h = self._flagged_node_pool(node_h, batch.node_features[:, 14], batch.node_batch, batch.num_graphs)
        goal_h = self._flagged_node_pool(node_h, batch.node_features[:, 13], batch.node_batch, batch.num_graphs)
        return torch.cat([global_h, mean_pool, max_pool, agent_h, goal_h], dim=-1)

    def _mean_pool(self, x: torch.Tensor, batch_index: torch.Tensor, num_graphs: int) -> torch.Tensor:
        out = torch.zeros((num_graphs, x.shape[-1]), dtype=x.dtype, device=x.device)
        out.index_add_(0, batch_index, x)
        counts = torch.zeros((num_graphs, 1), dtype=x.dtype, device=x.device)
        counts.index_add_(0, batch_index, torch.ones((x.shape[0], 1), dtype=x.dtype, device=x.device))
        return out / counts.clamp_min(1.0)

    def _max_pool(self, x: torch.Tensor, batch_index: torch.Tensor, num_graphs: int) -> torch.Tensor:
        pooled = []
        for graph_idx in range(num_graphs):
            values = x[batch_index == graph_idx]
            if values.numel() == 0:
                pooled.append(torch.zeros((x.shape[-1],), dtype=x.dtype, device=x.device))
            else:
                pooled.append(values.max(dim=0).values)
        return torch.stack(pooled, dim=0)

    def _flagged_node_pool(
        self,
        x: torch.Tensor,
        flags: torch.Tensor,
        batch_index: torch.Tensor,
        num_graphs: int,
    ) -> torch.Tensor:
        pooled = []
        for graph_idx in range(num_graphs):
            in_graph = batch_index == graph_idx
            values = x[in_graph]
            graph_flags = flags[in_graph]
            selected = values[graph_flags > 0.5]
            if selected.numel() == 0:
                pooled.append(torch.zeros((x.shape[-1],), dtype=x.dtype, device=x.device))
            else:
                pooled.append(selected.mean(dim=0))
        return torch.stack(pooled, dim=0)

    def forward(self, batch: GraphBatch) -> dict[str, torch.Tensor]:
        graph_h = self.encode_graph(batch)
        raw_mean = self.actor(graph_h) + self.global_actor(batch.global_features) + self._guidance_raw_prior(batch)
        value = self.critic(graph_h).squeeze(-1)
        return {"mean": torch.tanh(raw_mean) * self.max_acc, "value": value}

    def distribution(self, batch: GraphBatch) -> tuple[Normal, torch.Tensor]:
        graph_h = self.encode_graph(batch)
        raw_mean = self.actor(graph_h) + self.global_actor(batch.global_features) + self._guidance_raw_prior(batch)
        value = self.critic(graph_h).squeeze(-1)
        std = torch.exp(self.effective_log_std()).expand_as(raw_mean)
        return Normal(raw_mean, std), value

    def _zero_actor_head(self) -> None:
        self._zero_head(self.actor)

    @staticmethod
    def _zero_head(module_stack: nn.Sequential) -> None:
        for module in reversed(module_stack):
            if isinstance(module, nn.Linear):
                nn.init.zeros_(module.weight)
                nn.init.zeros_(module.bias)
                return

    def _guidance_raw_prior(self, batch: GraphBatch) -> torch.Tensor:
        if batch.global_features.shape[-1] < 20:
            return torch.zeros((batch.num_graphs, 2), dtype=batch.global_features.dtype, device=batch.global_features.device)
        if batch.global_features.shape[-1] >= 26:
            scaled_action = torch.clamp(batch.global_features[:, 24:26], -0.95, 0.95)
            return 0.5 * (torch.log1p(scaled_action) - torch.log1p(-scaled_action))
        velocity = batch.global_features[:, 2:4]
        guidance = batch.global_features[:, 16:18]
        gate_wait = torch.clamp(batch.global_features[:, 19:20], 0.0, 1.0)
        if batch.global_features.shape[-1] >= 22:
            target_dist = batch.global_features[:, 20:21]
            next_gate_wait = torch.clamp(batch.global_features[:, 21:22], 0.0, 1.0)
        else:
            target_dist = torch.ones_like(gate_wait)
            next_gate_wait = gate_wait
        if batch.global_features.shape[-1] >= 24:
            closed_gate_dist = batch.global_features[:, 22:23]
            closed_gate_wait = torch.clamp(batch.global_features[:, 23:24], 0.0, 1.0)
        else:
            closed_gate_dist = torch.ones_like(gate_wait)
            closed_gate_wait = torch.zeros_like(gate_wait)
        has_guidance = (guidance.norm(dim=-1, keepdim=True) > 1e-6).to(guidance.dtype)
        waypoint_speed = torch.clamp(target_dist * 12.0, 0.05, 1.0)
        wait_close = torch.clamp((0.12 - target_dist) / 0.12, 0.0, 1.0) * next_gate_wait
        front_closed = torch.clamp((0.18 - closed_gate_dist) / 0.18, 0.0, 1.0) * closed_gate_wait
        near_gate_brake = torch.clamp(1.0 - 2.5 * wait_close, 0.0, 1.0)
        front_gate_brake = torch.clamp(1.0 - 3.0 * front_closed, 0.0, 1.0)
        target_speed = waypoint_speed * near_gate_brake * front_gate_brake
        desired_velocity = guidance * target_speed - guidance * (0.35 * wait_close + 0.45 * front_closed)
        scaled_action = torch.clamp(self.guidance_prior_gain * (desired_velocity - velocity), -0.95, 0.95) * has_guidance
        return 0.5 * (torch.log1p(scaled_action) - torch.log1p(-scaled_action))

    def effective_log_std(self) -> torch.Tensor:
        return torch.clamp(self.log_std, self.min_log_std, self.max_log_std)

    def _squash(self, raw_action: torch.Tensor) -> torch.Tensor:
        return torch.tanh(raw_action) * self.max_acc

    def _atanh_scaled_action(self, action: torch.Tensor) -> torch.Tensor:
        scaled = torch.clamp(action / self.max_acc, -1.0 + self._eps, 1.0 - self._eps)
        return 0.5 * (torch.log1p(scaled) - torch.log1p(-scaled))

    def _squashed_log_prob(self, dist: Normal, raw_action: torch.Tensor) -> torch.Tensor:
        scaled = torch.tanh(raw_action)
        log_det = torch.log(self.max_acc * (1.0 - scaled.pow(2)) + self._eps)
        return (dist.log_prob(raw_action) - log_det).sum(dim=-1)

    def act(self, batch: GraphBatch, deterministic: bool = False) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist, value = self.distribution(batch)
        raw_action = dist.mean if deterministic else dist.rsample()
        action = self._squash(raw_action)
        log_prob = self._squashed_log_prob(dist, raw_action)
        return action, log_prob, value

    def evaluate_actions(self, batch: GraphBatch, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist, value = self.distribution(batch)
        raw_action = self._atanh_scaled_action(actions)
        log_prob = self._squashed_log_prob(dist, raw_action)
        entropy = dist.entropy().sum(dim=-1).mean()
        return log_prob, entropy, value


TeacherActorCritic = GNNTeacherActorCritic
