#!/bin/bash
python3 "$(dirname "$0")"/submerge_forward_10ft.py --depth 2 --yes
sleep 3
python3 "$(dirname "$0")"/check_horizontal_direction.py --strength 50 --duration 10 --together --yes
