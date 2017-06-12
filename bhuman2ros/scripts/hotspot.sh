#!/usr/bin/env bash
# This file enables/disables a hotspot

# if the hotspot name is not set, exit
if [ -z "$1" ]; then
  echo "Usage: $0 $1 {on|off}"
  exit 1
fi
 
# create hotspot variable and enable or disable hotspot with nmcli
HOTSPOT="$1"

case $2 in
  on)
    echo "Enabling hotspot $HOTSPOT..."
    if ! sudo nmcli connection up $HOTSPOT; then
      echo "Creating hotspot $HOTSPOT"
      sudo nmcli device wifi hotspot con-name $HOTSPOT ssid $HOTSPOT band bg password odroid123
    fi
    echo "$HOTSPOT enabled."
    ;;
  off)
    echo "Disabling hotspot $HOTSPOT..."
    sudo nmcli connection down $HOTSPOT
    echo "$HOTSPOT disabled."
    ;;
  *)
    echo "Usage: $0 $1 {on|off}"
    exit 1
esac
