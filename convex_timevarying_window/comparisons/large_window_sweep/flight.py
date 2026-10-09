"""Reuse unmodified current planner/controller; replace only the course geometry."""
from . import scenario
from .worker import Worker
from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight as current


def main():
    current.Worker=Worker
    current.build_case=scenario.build_case
    current.launch=scenario.launch
    current.manifest=scenario.manifest
    current.main()


if __name__=='__main__':main()
