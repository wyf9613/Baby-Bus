"""Dispatch the worker before importing ROS, preserving the single-file CLI."""
import sys


def main():
    # Tiny MPC/RNN matrices: avoid nested BLAS thread pools competing with ROS
    # and the planner process. Set BEFORE importing NumPy through node/learned.
    import os
    for name in ("OPENBLAS_NUM_THREADS","OMP_NUM_THREADS","MKL_NUM_THREADS","BLIS_NUM_THREADS"):
        os.environ[name]="1"
    if '--shadow-worker' in sys.argv:
        from mvp21_policy.worker import run_worker
        run_worker()
    else:
        from mvp21_policy.node import main as ros_main
        ros_main()


if __name__ == '__main__':
    main()
