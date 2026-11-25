from . import scenario
from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight as current,px4
from convex_timevarying_window.comparisons.large_window_sweep.worker import Worker


def main():
    current.build_case=scenario.build_case;current.launch=scenario.launch
    current.manifest=scenario.manifest;current.Worker=Worker;current.MAX_FLIGHT_TIME=180
    px4.WORKSPACE_LOWER=scenario.LOWER;px4.WORKSPACE_UPPER=scenario.UPPER
    current.main()


if __name__=='__main__':main()
