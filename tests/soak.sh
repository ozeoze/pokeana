#!/bin/bash
# Long run: ADALM2000 drives 4 EEPROM buses; the GUI captures in Repeat mode and dumps its decodes.
D=~/soak; mkdir -p $D; cd ~/Claude/pokeana
HOURS=${1:-12}; SECS=$((HOURS * 3600))
date > $D/started
nohup timeout $((SECS + 600)) python3 -u tests/m2k_eeprom.py gen $((SECS + 300)) > $D/gen.log 2>&1 &
sleep 3
GDK_BACKEND=broadway BROADWAY_DISPLAY=:5 POKEANA_GUI_DUMP=$D/dump.jsonl POKEANA_GUI_SHOT=$D/gui.png \
POKEANA_GUI_DO="rate 10000000;depth 1000000;pre 5;show 18;place 1;cond 10 f;dec i2c:scl=9,sda=10,layer=eeprom;dec i2c:scl=12,sda=11,layer=eeprom;dec i2c:scl=15,sda=14,layer=eeprom;dec i2c:scl=17,sda=16,layer=eeprom;list;repeat $SECS 1800" \
nohup timeout $((SECS + 300)) python3 pokeana_gui.py > $D/gui.log 2>&1 &
GUI=$!
# every 30 min: the check so far and the GUI's memory
while kill -0 $GUI 2>/dev/null; do
  sleep 1800
  P=$(pgrep -f "^python3 pokeana_gui.py" | head -1)
  echo "$(date +%H:%M) rss_kb=$(ps -o rss= -p $P 2>/dev/null) $(python3 tests/m2k_eeprom.py check $D/dump.jsonl | head -1)" >> $D/progress.log
done
python3 tests/m2k_eeprom.py check $D/dump.jsonl > $D/final.txt
date > $D/finished
