#!/bin/bash

set -euo pipefail

OUTDIR="${1:-results/fixed_v2_first_solution_$(date +%Y%m%d_%H%M%S)}"
TERMINATION_MODE="${TERMINATION_MODE:-first_solution}"
TIME_LIMIT_PER_LEG="${TIME_LIMIT_PER_LEG:-1.0}"

mkdir -p "$OUTDIR"

# Choose 5 environment seeds
for env in 0 1 2 3 4; do
  # 20 stochastic trials per environment
  for r in $(seq 0 19); do
    # Same seed for all planners in this (env, run)
    seed=$((12345 + 100000*env + 1000*r))

    for planner in RRTConnect PRMstar RRTstar BITstar; do
      ./se3_room_tour_one_run \
        --planner "$planner" \
        --seed "$seed" \
        --env_seed "$env" \
        --run "$r" \
        --outdir "$OUTDIR" \
        --termination_mode "$TERMINATION_MODE" \
        --time_limit_per_leg "$TIME_LIMIT_PER_LEG"
    done
  done
done

echo "All runs completed. Results are in: $OUTDIR"
