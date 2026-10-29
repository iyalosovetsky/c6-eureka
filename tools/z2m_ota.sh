#!/bin/sh
# Zigbee OTA for c6 through zigbee2mqtt. Run as root on the Home Assistant host
# (rpi516g3), after copying the .ota file and c6-index.json from make_ota.py to
# $Z2M/ota/.
#
#   sudo sh z2m_ota.sh check     # does z2m see a newer image?
#   sudo sh z2m_ota.sh update    # flash it (~80 min), then fan back on
#
# Uses z2m's default 50-byte blocks: 100 makes the device abort the transfer.
# After the update c6 reboots with the fan off; FAN_SPEED (default 40 %) turns
# it back on.
set -e
Z2M=/var/lib/homeassistant/homeassistant/zigbee2mqtt
O=/var/lib/homeassistant/apps/data/45df7312_zigbee2mqtt/options.json
M=app_core_mosquitto
PW=$(python3 -c "import json; print(json.load(open('$O'))['mqtt']['password'])")
U=$(python3 -c "import json; print(json.load(open('$O'))['mqtt']['user'])")
pub() { docker exec $M mosquitto_pub -h localhost -u "$U" -P "$PW" -t "$1" -m "$2"; }

case "$1" in
  check) wait_s=120 ;;
  update) wait_s=10800 ;;
  *) echo "usage: $0 check|update" >&2; exit 2 ;;
esac
[ -f "$Z2M/ota/c6-index.json" ] || { echo "no $Z2M/ota/c6-index.json" >&2; exit 1; }

docker exec $M timeout $wait_s mosquitto_sub -h localhost -u "$U" -P "$PW" -C 1 \
  -t "zigbee2mqtt/bridge/response/device/ota_update/$1" > /tmp/c6_ota_result &
sleep 1
pub "zigbee2mqtt/bridge/request/device/ota_update/$1" '{"id":"c6","url":"ota/c6-index.json"}'
wait
cat /tmp/c6_ota_result

if [ "$1" = update ] && grep -q '"status":"ok"' /tmp/c6_ota_result; then
  sleep 45
  pub zigbee2mqtt/c6/set "{\"state_fan_on_off\":\"ON\",\"fan_set_speed\":${FAN_SPEED:-40}}"
  echo "fan back on at ${FAN_SPEED:-40} %"
fi
