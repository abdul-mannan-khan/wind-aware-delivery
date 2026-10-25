# Printed when you open a terminal in the container.
[ -n "$QWC_WELCOMED" ] && return 0 2>/dev/null; export QWC_WELCOMED=1
cat <<'TXT'

  Quadrotor delivery through urban wind corridors
  -----------------------------------------------
  Code:        ~/wind-aware-delivery        (update with: git pull)
  Smoke test:  cd ~/wind-aware-delivery/sim && python3 run_experiments.py --config ../configs/quick.yaml
  One mission: python3 fly_one.py --U 12 --theta 180 --planner wind
  Summary:     python3 summarise.py ../results/experiments/quick/campaign.csv
  AirSim:      start_airsim   (then: python3 airsim_replay.py --one)   stop with: stop_airsim
  Guides:      docs/ in the repository (or on GitHub)

TXT
