#!/usr/bin/env bash
# Mac host helper. All ROS work happens in this project's Linux container.
set -euo pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
profile=babybus-jazzy
context=colima-babybus-jazzy
container=babybus-jazzy
image=babybus-ros-jazzy:local
evidence="$task_root/.verification/ros2-jazzy"
action="${1:-shell}"
mkdir -p "$evidence"

docker_local() { docker --context "$context" "$@"; }

start_vm() {
  if ! colima status --profile "$profile" >/dev/null 2>&1; then
    colima start --profile "$profile" --activate=false --ssh-config=false
  fi
}

ensure_container() {
  start_vm
  docker_local image inspect "$image" >/dev/null
  if docker_local container inspect "$container" >/dev/null 2>&1; then
    actual_root="$(docker_local inspect --format '{{index .Config.Labels "org.babybus.source"}}' "$container")"
    [[ "$actual_root" == "$task_root" ]] || { echo 'Container belongs to a different checkout.' >&2; exit 1; }
    if [[ "$(docker_local inspect --format '{{.State.Running}}' "$container")" != true ]]; then
      docker_local start "$container" >/dev/null
    fi
  else
    git_common="$(git -C "$task_root" rev-parse --path-format=absolute --git-common-dir)"
    mount_args=(--mount "type=bind,source=$task_root,target=$task_root")
    # The independent worktree's .git file points to its original repository.
    # Mount those metadata read-only so ordinary Git reads also work in ROS.
    if [[ "$git_common" != "$task_root/.git" ]]; then
      mount_args+=(--mount "type=bind,source=$git_common,target=$git_common,readonly")
    fi
    docker_local run --detach --init --name "$container" \
      --label "org.babybus.source=$task_root" \
      "${mount_args[@]}" \
      --mount "type=volume,source=babybus-jazzy-verification,target=$task_root/.verification/ws" \
      --workdir "$task_root" \
      --env "AI4R_POLICY_SOURCE=$task_root" \
      --env "AI4R_INTERFACES_SOURCE=$task_root/.verification/dependencies/dream_interfaces" \
      "$image" >/dev/null
  fi
  # The container is root; explicitly trust only these local source checkouts.
  docker_local exec "$container" git config --global --replace-all safe.directory "$task_root"
  docker_local exec "$container" git config --global --add safe.directory \
    "$task_root/.verification/dependencies/dream_interfaces"
}

case "$action" in
  build)
    start_vm
    base="$(cat "$evidence/base-image-ref.txt")"
    docker_local build --build-arg "ROS_BASE_IMAGE=$base" --tag "$image" "$task_root/tools/ros2_jazzy"
    ;;
  start)
    ensure_container
    echo 'Baby-Bus ROS Jazzy container is running.'
    ;;
  shell)
    ensure_container
    exec docker --context "$context" exec -it "$container" bash --rcfile /opt/babybus/shell.bashrc -i
    ;;
  check)
    ensure_container
    run_dir="$evidence/checks/$(date +%Y%m%d-%H%M%S)"
    mkdir -p "$run_dir"
    set +e
    docker_local exec "$container" bash tools/verify_fast.sh 2>&1 | tee "$run_dir/verify-fast.log"
    result=${PIPESTATUS[0]}
    set -e
    docker_local cp "$container:$task_root/.verification/ws/log" "$run_dir/colcon-log" || true
    docker_local cp "$container:$task_root/.verification/ws/build/ai4r_policy/test_results" "$run_dir/test-results" || true
    printf '%s\n' "$result" > "$run_dir/exit-code.txt"
    echo "ROS check log: $run_dir/verify-fast.log"
    exit "$result"
    ;;
  status)
    colima status --profile "$profile"
    docker_local ps --filter "name=^/${container}$"
    ;;
  lidar-stop)
    ensure_container
    run_dir="$task_root/.verification/ros-lidar-stop/runs/$(date +%Y%m%d-%H%M%S)"
    mkdir -p "$(dirname "$run_dir")"
    docker_local exec "$container" bash -ec \
      'source .verification/ws/install/setup.bash; exec python3 tools/ros2_jazzy/lidar_stop_demo.py --output "$1"' \
      bash "$run_dir" 2>&1 | tee "$run_dir.log"
    echo "ROS LiDAR result: $run_dir/summary.json"
    ;;
  stop)
    if docker_local container inspect "$container" >/dev/null 2>&1; then
      docker_local stop "$container"
    fi
    colima stop --profile "$profile"
    ;;
  *)
    echo 'Usage: tools/ros2_jazzy/run.sh {shell|start|check|lidar-stop|status|stop|build}' >&2
    exit 2
    ;;
esac
