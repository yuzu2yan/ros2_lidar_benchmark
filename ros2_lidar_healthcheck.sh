#!/usr/bin/env bash
set -euo pipefail

OUTDIR="/var/log/ros2-lidar-bench/health"
mkdir -p "$OUTDIR"
DAY="$(date +%F)"
OUT="$OUTDIR/$DAY.txt"

{
  echo "=== Health Report: $DAY ==="
  echo "# Uptime / Reboot:"
  who -b || true
  echo

  echo "# Kernel / Reboot markers (last 3 boots):"
  last -x | egrep 'reboot|shutdown' | head -n 20 || true
  echo

  echo "# Critical journal (since yesterday):"
  journalctl -p 0..3 --since=yesterday --no-pager || true
  echo

  echo "# Suspected issues (since yesterday):"
  journalctl --since=yesterday --no-pager | egrep -i \
    'oom|out of memory|watchdog|segfault|backtrace|panic|GPU hang|I/O error|fatal|emergency' || true
  echo

  echo "# CPU temp/clock summary (yesterday's CSV if present):"
  # If yesterday's sys_temp_clock.csv exists, print the maximum/average temperature and clock
  YDAY="$(date -d 'yesterday' +%F)"
  CSV="/var/log/ros2-lidar-bench/$YDAY/sys_temp_clock.csv"
  if [[ -f "$CSV" ]]; then
    awk -F, 'NR>1 {tc+=$2; hz+=$3; n++; if($2>tmax)tmax=$2; if($3>hzmax)hzmax=$3} END{if(n) printf("avg_temp=%.2fC max_temp=%.2fC avg_arm=%.0fHz max_arm=%.0fHz\n", tc/n, tmax, hz/n, hzmax); else print "no data"}' "$CSV"
  else
    echo "no CSV for $YDAY"
  fi
} > "$OUT"

# If there are any suspicious issues, exit with code 1 (for CI etc.)
if grep -qiE 'oom|out of memory|watchdog|segfault|panic|fatal|emergency' "$OUT"; then
  exit 1
fi

 # sudo chmod +x /usr/local/bin/ros2_lidar_healthcheck.sh