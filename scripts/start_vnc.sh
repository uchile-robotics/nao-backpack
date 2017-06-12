#!/usr/bin/env bash
# This file runs a VNC server

sudo x11vnc -xkb -noxrecord -noxfixes -noxdamage -display :0 -auth /var/run/lightdm/root/:0 -usepw -ncache 10

