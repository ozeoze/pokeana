#!/bin/sh
# volt_step.sh label: OK the open threshold dialog, Run, Stop; times to cap/08_times.txt
cd ~/vm
echo "$(date +%s.%N) ok $1" >> cap/08_times.txt; python3 click.py 247 300 >/dev/null; sleep 4
echo "$(date +%s.%N) run $1" >> cap/08_times.txt; python3 click.py 178 50 >/dev/null; sleep 6
python3 click.py 247 50 >/dev/null; sleep 6
echo "$(date +%s.%N) done $1" >> cap/08_times.txt
python3 qmp.py screendump /tmp/s.png >/dev/null
