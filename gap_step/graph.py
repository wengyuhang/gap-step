from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch


GLOBAL_FEATURE_DIM = 26
NODE_FEATURE_DIM = 32
EDGE_FEATURE_DIM = 20


@dataclass(frozen=True)
class GraphObs:
    global_features: np.ndarray
    node_features: np.ndarray
    node_type: np.ndarray
    edge_index: np.ndarray
    edge_features: np.ndarray


@dataclass(frozen=True)
class GraphBatch:
    global_features: torch.Tensor
    node_features: torch.Tensor
    node_type: torch.Tensor
    edge_index: torch.Tensor
    edge_features: torch.Tensor
    node_batch: torch.Tensor

    @property
    def num_graphs(self) -> int:
        return int(self.global_features.shape[0])


def collate_graph_obs(obs_list: list[GraphObs], device: torch.device) -> GraphBatch:
    if not obs_list:
        raise ValueError("Cannot collate an empty graph observation list")

    global_features = torch.as_tensor(np.stack([obs.global_features for obs in obs_list]), dtype=torch.float32, device=device)
    node_features_parts = []
    node_type_parts = []
    edge_features_parts = []
    edge_index_parts = []
    node_batch_parts = []
    node_offset = 0
    for graph_idx, obs in enumerate(obs_list):
        node_count = int(obs.node_features.shape[0])
        edge_count = int(obs.edge_features.shape[0])
        node_features_parts.append(torch.as_tensor(obs.node_features, dtype=torch.float32, device=device))
        node_type_parts.append(torch.as_tensor(obs.node_type, dtype=torch.long, device=device))
        edge_features_parts.append(torch.as_tensor(obs.edge_features, dtype=torch.float32, device=device))
        edge_index = torch.as_tensor(obs.edge_index, dtype=torch.long, device=device)
        if edge_count:
            edge_index_parts.append(edge_index + node_offset)
        node_batch_parts.append(torch.full((node_count,), graph_idx, dtype=torch.long, device=device))
        node_offset += node_count

    node_features = torch.cat(node_features_parts, dim=0)
    node_type = torch.cat(node_type_parts, dim=0)
    edge_features = torch.cat(edge_features_parts, dim=0)
    edge_index = torch.cat(edge_index_parts, dim=1) if edge_index_parts else torch.zeros((2, 0), dtype=torch.long, device=device)
    node_batch = torch.cat(node_batch_parts, dim=0)
    return GraphBatch(
        global_features=global_features,
        node_features=node_features,
        node_type=node_type,
        edge_index=edge_index,
        edge_features=edge_features,
        node_batch=node_batch,
    )
