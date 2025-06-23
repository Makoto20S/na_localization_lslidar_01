#!/bin/bash
#gnome-terminal -t "source" -x bash -c "source /home/ywb/NR_mapping/devel/setup.bash;exec bash"

#sleep 1s

#gnome-terminal -t "savemap" -x bash -c "rosservice call save_map 1.0 '';exec bash"
source /home/firefly/na_localization/devel/setup.bash
rosservice call save_map 1.0 ''
