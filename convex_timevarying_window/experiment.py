"""Compatibility entry point for the TOGT baseline."""

from convex_timevarying_window.togt.experiment import *  # noqa: F401,F403
from convex_timevarying_window.togt.experiment import main


if __name__ == "__main__":
    raise SystemExit(main())
