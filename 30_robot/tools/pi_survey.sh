#!/usr/bin/env bash
# Read-only survey of the cart Raspberry Pi. Changes nothing.
# Run on the Pi:  bash pi_survey.sh > pi_survey_$(hostname).txt 2>&1
section() { echo; echo "===== $1 ====="; }

section "date / host"
date; hostname; hostname -I
section "hardware"
cat /proc/device-tree/model 2>/dev/null; echo
lscpu | grep -E "Architecture|Model name|CPU\(s\)"
uname -a
free -h
df -h /
section "os"
cat /etc/os-release
getconf LONG_BIT
section "network"
ip -brief addr
iwgetid 2>/dev/null || nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null | grep '^yes'
ping -c 2 -W 2 8.8.8.8 >/dev/null 2>&1 && echo "internet: ping OK" || echo "internet: ping FAILED"
curl -sS -o /dev/null -w "app.viam.com HTTP %{http_code}\n" --max-time 8 https://app.viam.com || echo "app.viam.com: NOT reachable"
section "interfaces"
ls -l /dev/i2c* /dev/spidev* /dev/serial* /dev/ttyAMA* /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
grep -vE '^\s*#|^\s*$' /boot/firmware/config.txt 2>/dev/null || grep -vE '^\s*#|^\s*$' /boot/config.txt 2>/dev/null
cat /boot/firmware/cmdline.txt 2>/dev/null || cat /boot/cmdline.txt 2>/dev/null
command -v i2cdetect >/dev/null && { echo "i2c bus 1 (MPU6050 is usually 0x68):"; i2cdetect -y 1; } || echo "i2cdetect not installed"
section "usb devices (lidar, cameras, adapters)"
lsusb 2>/dev/null
ls /dev/video* 2>/dev/null
section "gpio tools"
for t in pinctrl raspi-gpio gpio pigpiod; do command -v $t >/dev/null && echo "has $t"; done
command -v pinctrl >/dev/null && pinctrl get 14,15,18,13,19,26 2>/dev/null
command -v raspi-gpio >/dev/null && raspi-gpio get 14,15,18,13,19,26 2>/dev/null
section "ros"
ls /opt/ros 2>/dev/null || echo "no /opt/ros"
env | grep -i ros
section "what runs at boot / now"
systemctl list-units --type=service --state=running --no-pager --no-legend | awk '{print $1}'
crontab -l 2>/dev/null; sudo -n crontab -l 2>/dev/null
ls /etc/systemd/system/*.service 2>/dev/null
grep -vE '^\s*#|^\s*$' /etc/rc.local 2>/dev/null
section "old project code (look for the cart control)"
ls -la ~ 2>/dev/null
find / -xdev \( -iname '*prototype*' -o -iname '*mauro*' -o -iname '*joy_controller*' -o -iname 'colcon_ws' -o -iname '*_ws' -o -iname 'catkin_ws' \) -maxdepth 5 2>/dev/null | head -30
section "users"
getent passwd | awk -F: '$3>=1000 && $3<60000 {print $1, $6}'
section "viam"
command -v viam-server && viam-server --version 2>/dev/null; ls /etc/viam.json 2>/dev/null
echo; echo "===== survey done ====="
