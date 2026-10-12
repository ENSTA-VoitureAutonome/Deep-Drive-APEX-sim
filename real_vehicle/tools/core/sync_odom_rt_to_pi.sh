#!/usr/bin/env bash
# Ship the learned odometry to the car, with a manifest.
#
# The estimator does not travel with the rest of the code and that is
# deliberate. It needs torch, onnxruntime and numba -- about 1 GB that will not
# fit in the Docker image on a disk with 1.4 GB free -- and it is *coupled to
# the trained weights*: the de-skewing, the metric embedding and the ICP are the
# network's input pipeline, so a divergent copy would silently invalidate the
# model without failing anything. So it lives in ~/odom_rt on the Pi, beside the
# python libraries, and the container mounts it read-only.
#
# This script sends an explicit file list -- never a directory sweep -- and
# writes a sha256 manifest plus the git commit, so what is running on the car
# can always be traced back to a revision of this repository.
#
#   ./real_vehicle/tools/core/sync_odom_rt_to_pi.sh
#   ssh raspberrypi 'cd ~/odom_rt && sha256sum -c MANIFEST.sha256'
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
REMOTE="${APEX_PI_HOST:-raspberrypi}"
REMOTE_DIR="${APEX_ODOM_RT_DIR:-/home/ensta/odom_rt}"
CKPT_DIR="${APEX_ODOM_CKPT_DIR:-simulation/learning/outputs/real2sim_v5_10hz/hybrid_submap_v4}"

# The estimator's runtime: every module live_odometry imports, and nothing else.
FILES=(
  # The estimator's import closure, computed from learned_odometry_node.py and
  # not by hand: `sensors.py` reaches into pose_dataset.raycast (and so into
  # geometry and tracks) and into apex_fusion_research.core, all at module
  # level, so a missing one is not a degraded estimator but a node that will
  # not start. Shipping them does not weaken the knowledge firewall: /odom_rt
  # is the estimator's tree and is not on the driver's PYTHONPATH, and
  # test_knowledge_firewall.py polices the driver's package.
  #
  # The layout mirrors the repository because sensors.py locates
  # apex_fusion_research relative to its own file.
  simulation/learning/lidar_imu_pose/learned_odometry_node.py
  simulation/learning/lidar_imu_pose/live_odometry.py
  simulation/learning/lidar_imu_pose/standstill.py
  simulation/learning/lidar_imu_pose/streaming_model.py
  simulation/learning/lidar_imu_pose/icp_odometry.py
  simulation/learning/lidar_imu_pose/polar_icp.py
  simulation/learning/lidar_imu_pose/onnx_net.py
  simulation/learning/lidar_imu_pose/sqlite_streams.py
  simulation/learning/lidar_imu_pose/sqlite_windows.py
  simulation/learning/lidar_imu_pose/train_pose_fusion.py
  simulation/learning/lidar_imu_pose/latency_profile.py
  simulation/tools/pose_dataset/__init__.py
  simulation/tools/pose_dataset/sensors.py
  simulation/tools/pose_dataset/blobs.py
  simulation/tools/pose_dataset/raycast.py
  simulation/tools/pose_dataset/geometry.py
  simulation/tools/pose_dataset/tracks.py
  simulation/tools/pose_dataset/config/sensors.yaml
  simulation/ros2_ws/src/apex_fusion_research/apex_fusion_research/__init__.py
  simulation/ros2_ws/src/apex_fusion_research/apex_fusion_research/core/__init__.py
  simulation/ros2_ws/src/apex_fusion_research/apex_fusion_research/core/config_io.py
  simulation/ros2_ws/src/apex_fusion_research/apex_fusion_research/core/imu_error.py
  simulation/ros2_ws/src/apex_fusion_research/apex_fusion_research/core/lidar_noise.py
  "${CKPT_DIR}/best_model.pt"
  "${CKPT_DIR}/best_model.onnx"
)

cd "${REPO_ROOT}"
for f in "${FILES[@]}"; do
  [[ -f "${f}" ]] || { echo "missing: ${f}" >&2; exit 1; }
done

echo "[odom_rt] syncing $(( ${#FILES[@]} )) files to ${REMOTE}:${REMOTE_DIR}"
printf '%s\n' "${FILES[@]}" | rsync -av --checksum --files-from=- --no-perms --no-owner --no-group \
  ./ "${REMOTE}:${REMOTE_DIR}/"

COMMIT="$(git rev-parse HEAD)"
DIRTY="$(git status --porcelain -- "${FILES[@]}" | wc -l | tr -d ' ')"
{
  printf '# apex learned odometry deployed from %s\n' "${REPO_ROOT}"
  printf '# git %s%s on %s\n' "${COMMIT}" "$([[ "${DIRTY}" != "0" ]] && echo ' (WITH UNCOMMITTED CHANGES)')" "$(date -Is)"
  sha256sum "${FILES[@]}"
} > /tmp/apex_odom_manifest.sha256
rsync -a /tmp/apex_odom_manifest.sha256 "${REMOTE}:${REMOTE_DIR}/MANIFEST.sha256"

echo "[odom_rt] verifying on the car"
ssh "${REMOTE}" "cd ${REMOTE_DIR} && sha256sum -c MANIFEST.sha256 --quiet && echo '[odom_rt] manifest OK'"
if [[ "${DIRTY}" != "0" ]]; then
  echo "[odom_rt] WARNING: ${DIRTY} of the deployed files have uncommitted changes" >&2
fi

# Import the node for real, in the container that will run it. A missing
# transitive import is invisible to rsync and to the manifest, and it does not
# surface until the stack is already up and the watchdog tears everything down.
if [[ "${APEX_ODOM_VERIFY:-1}" == "1" ]]; then
  echo "[odom_rt] importing the estimator in the container"
  ssh "${REMOTE}" "export PATH=\$HOME/local/bin:\$PATH; docker run --rm -v ${REMOTE_DIR}:/odom_rt:ro docker-apex_pipeline \
    bash -lc 'source /opt/ros/jazzy/setup.bash && cd /odom_rt/simulation/learning/lidar_imu_pose && \
      PYTHONPATH=/odom_rt/pylib:/odom_rt/pylib_b:\$PYTHONPATH python3 -c \"
import sys, pathlib; sys.path.insert(0, \\\"/odom_rt/simulation/tools\\\")
from pose_dataset.sensors import load_sensor_profiles
import live_odometry, polar_icp, onnx_net, streaming_model
p = load_sensor_profiles(pathlib.Path(\\\"/odom_rt/simulation/tools/pose_dataset/config/sensors.yaml\\\"))
print(\\\"[odom_rt] import OK; profiles:\\\", len(p), \\\"APEX_real_wt61 IMU\\\", p[\\\"APEX_real_wt61\\\"][\\\"imu\\\"][\\\"rate_hz\\\"], \\\"Hz\\\")
\"'"
fi
