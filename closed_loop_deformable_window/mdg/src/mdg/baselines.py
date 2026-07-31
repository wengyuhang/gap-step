"""Shared baseline identifiers and semantics."""

METHODS = (
    "center",
    "largest_disc",
    "uniform_point",
    "mdg_center",
    "mdg_free",
    "dense_oracle",
)

FREE_POINT_METHODS = {"largest_disc", "mdg_free", "dense_oracle"}
NON_ORACLE_METHODS = METHODS[:-1]


def normalize_method(value: str) -> str:
    normalized = value.lower().replace("-", "_")
    if normalized not in METHODS:
        raise ValueError(f"unknown method {value!r}; choose from {METHODS}")
    return normalized


__all__ = [
    "FREE_POINT_METHODS",
    "METHODS",
    "NON_ORACLE_METHODS",
    "normalize_method",
]

