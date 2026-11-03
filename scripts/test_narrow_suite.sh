#!/usr/bin/env bash
set -eo pipefail
INDOOR_SUITE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$INDOOR_SUITE_ROOT"
./scripts/test_detour_footprint.sh
./scripts/validate.sh --narrow-isolated --world results/narrow_passage/gap_0.45.world \
  --output results/narrow_passage/final_045 --goal 12 0 --timeout 18 --expect-blocked
./scripts/validate.sh --narrow-isolated --world results/narrow_passage/gap_0.70.world \
  --output results/narrow_passage/final_070 --goal 12 0 --timeout 60
./scripts/validate.sh --narrow-isolated --world results/narrow_passage/live_scene.world \
  --output results/narrow_passage/final_scene
./scripts/validate.sh --avoidance-isolated --output results/narrow_passage/final_standard
