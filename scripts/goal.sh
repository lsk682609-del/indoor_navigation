#!/usr/bin/env bash
set -eo pipefail
if [[ "${1:-}" == '-h' || "${1:-}" == '--help' ]]; then
    echo '用法：./scripts/goal.sh X Y [--timeout 秒]'
    echo '示例：./scripts/goal.sh 3 0    # 去 map 坐标 (3, 0)，单位米'
    exit 0
fi
if [[ $# -lt 2 || "$2" == --* ]]; then
    echo '目标需要两个坐标，用空格分开：X Y。' >&2
    echo '示例：./scripts/goal.sh 3 0' >&2
    echo '3.0 只有一个数字；如需 X=3.0、Y=0，请输入：./scripts/goal.sh 3.0 0' >&2
    exit 2
fi
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec python3 "$INDOOR_ROOT/tests/navigation_smoke.py" --goal "$@"
