#!/usr/bin/env bash
set -euo pipefail

# Recover Lubancat 4 onboard RTL8852BE Wi-Fi after the driver gets stuck.
# Symptom seen on this board: nmcli scan returns zero APs and dmesg repeats
# "halbb_*_rf_reg_8852b_a is_w_busy/is_r_busy".

WIFI_DEV="${WIFI_DEV:-wlan0}"
CONNECTION_NAME="${CONNECTION_NAME:-}"
LOG_TAG="rtl8852be-wifi-watchdog"

log() {
  logger -t "$LOG_TAG" "$*"
  printf '%s\n' "$*"
}

reload_driver() {
  if systemctl list-unit-files rtl8852be-reload.service >/dev/null 2>&1; then
    systemctl restart rtl8852be-reload.service
  else
    modprobe -r 8852be
    modprobe -i 8852be
  fi
}

wifi_state() {
  nmcli -t -f DEVICE,TYPE,STATE device status \
    | awk -F: -v dev="$WIFI_DEV" '$1 == dev && $2 == "wifi" { print $3; exit }'
}

scan_count() {
  timeout 12 nmcli -t -f SSID dev wifi list --rescan yes 2>/dev/null \
    | sed '/^$/d' \
    | wc -l \
    | tr -d ' '
}

state="$(wifi_state || true)"

if [[ "$state" == "connected" ]]; then
  exit 0
fi

aps="$(scan_count || printf '0')"

if [[ -n "$state" && "$aps" != "0" ]]; then
  exit 0
fi

log "$WIFI_DEV state=${state:-missing} scan_count=$aps; reloading rtl8852be"

reload_driver
sleep 5

nmcli radio wifi on >/dev/null 2>&1 || true
nmcli device set "$WIFI_DEV" managed yes >/dev/null 2>&1 || true
nmcli device wifi rescan ifname "$WIFI_DEV" >/dev/null 2>&1 || true

if [[ -n "$CONNECTION_NAME" ]]; then
  nmcli connection up "$CONNECTION_NAME" >/dev/null 2>&1 || true
fi

new_state="$(wifi_state || true)"
new_aps="$(scan_count || printf '0')"
log "after recovery: $WIFI_DEV state=${new_state:-missing} scan_count=$new_aps"
