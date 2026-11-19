#!/usr/bin/env bash
# Copy arm/ to armster and (re)start the layout editor service there.
#
#   ./deploy.sh                    then open http://armster:8792 (over Tailscale)
#
# Installs deploy/homeostat-editor.service into /etc/systemd/system, enabled at boot.
# Plans made on the Pi stay on the Pi, in ~/homeostat-arm/arm/plans.
set -euo pipefail
HOST=${ARM_HOST:-pi@armster}
cd "$(dirname "$0")"

# Restarting the service ends whatever it is doing: a plan, or an air-run with the arm moving.
if [ "${FORCE:-}" != "1" ] && ssh "$HOST" 'curl -s -m 5 http://$(tailscale ip -4):8792/api/job' 2>/dev/null | grep -q '"state": "running"'; then
  echo "The editor is busy (a plan or an air-run is running). Try again when it's done, or FORCE=1 ./deploy.sh." >&2
  exit 1
fi

ssh "$HOST" 'mkdir -p ~/homeostat-arm/arm/plans'
scp -q arm/*.py arm/editor.html arm/viewer_template.html "$HOST:homeostat-arm/arm/"
scp -q deploy/homeostat-editor.service "$HOST:/tmp/homeostat-editor.service"
ssh "$HOST" 'sudo install -m 644 /tmp/homeostat-editor.service /etc/systemd/system/homeostat-editor.service \
  && rm /tmp/homeostat-editor.service \
  && sudo systemctl daemon-reload \
  && sudo systemctl enable --quiet homeostat-editor \
  && sudo systemctl restart homeostat-editor \
  && sleep 3 \
  && systemctl is-active homeostat-editor \
  && journalctl -u homeostat-editor -n 3 --no-pager -o cat'
