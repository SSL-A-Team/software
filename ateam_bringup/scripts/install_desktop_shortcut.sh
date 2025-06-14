#! /usr/bin/env bash

ateam_bringup_prefix=$(ros2 pkg prefix ateam_bringup)

cat <<EOF > ateam_bringup_physical.desktop
[Desktop Entry]
Version=1.0
Name=Bringup Physical
Exec=terminator --maximize --command "bash -c source /.bashrc; bash -i <<< 'ros2 launch ateam_bringup bringup_physical.launch.py; exec </dev/tty'"
Type=Application
Icon=$ateam_bringup_prefix/share/ateam_bringup/images/ateam_launcher_icon.png
EOF

chmod +x ateam_bringup_physical.desktop
desktop-file-install --delete-original --dir=$HOME/.local/share/applications ateam_bringup_physical.desktop
