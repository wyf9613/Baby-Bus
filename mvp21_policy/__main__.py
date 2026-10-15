"""Dispatch the worker before importing ROS, preserving the single-file CLI."""
import sys


def main():
    if '--shadow-worker' in sys.argv:
        from mvp21_policy.worker import run_worker
        run_worker()
    else:
        from mvp21_policy.node import main as ros_main
        ros_main()


if __name__ == '__main__':
    main()
