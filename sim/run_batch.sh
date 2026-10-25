#!/bin/bash
# Run several experiment configs one after another, then summarise them together.
# Usage (from sim/):  bash run_batch.sh ../configs/a.yaml ../configs/b.yaml ...
# Extra options for run_experiments.py can be given in WORKERS, e.g. WORKERS=8 bash run_batch.sh ...
set -uo pipefail
[ $# -ge 1 ] || { echo "usage: bash run_batch.sh CONFIG.yaml [CONFIG.yaml ...]"; exit 1; }
OUT=../results/experiments/batch_$(date +%Y%m%d_%H%M)
mkdir -p "$OUT"
LOG=$OUT/batch.log
CSVS=()
for cfg in "$@"; do
  name=$(python3 -c "import sys,yaml,os; c=yaml.safe_load(open(sys.argv[1])) or {}; print(c.get('name') or os.path.splitext(os.path.basename(sys.argv[1]))[0])" "$cfg")
  echo "=== $(date '+%H:%M:%S') $cfg -> $name" | tee -a "$LOG"
  if python3 run_experiments.py --config "$cfg" ${WORKERS:+--workers $WORKERS} 2>&1 | tee -a "$LOG"; then
    CSVS+=("../results/experiments/$name/campaign.csv")
  else
    echo "!!! $cfg FAILED, continuing with the next one" | tee -a "$LOG"
  fi
done
echo "=== $(date '+%H:%M:%S') summary" | tee -a "$LOG"
python3 summarise.py "${CSVS[@]}" --out "$OUT" 2>&1 | tee -a "$LOG"
echo "batch finished: $OUT"
