#!/bin/bash
# Long run: the NUC980 firmware (tests/nuc980_ebi.py load) loops on its EBI; the GUI captures
# in Repeat mode with the parallel decoder and dumps what it decoded, checked every 30 min.
D=~/soak_ebi; mkdir -p $D; cd ~/Claude/pokeana
HOURS=${1:-12}; SECS=$((HOURS * 3600))
date > $D/started
GDK_BACKEND=broadway BROADWAY_DISPLAY=:5 POKEANA_GUI_DUMP=$D/dump.jsonl POKEANA_GUI_SHOT=$D/gui.png \
POKEANA_GUI_DO="rate 100000000;depth 1000000;pre 1;show 25;place 1;cond 22 f;dec parallel:data=0-12,addr=18-21,cs=22,rd=23,wr=24;list;repeat $SECS 1800" \
nohup timeout $((SECS + 300)) python3 pokeana_gui.py > $D/gui.log 2>&1 &
GUI=$!
while kill -0 $GUI 2>/dev/null; do
  sleep 1800
  P=$(pgrep -f "^python3 pokeana_gui.py" | head -1)
  echo "$(date +%H:%M) rss_kb=$(ps -o rss= -p $P 2>/dev/null) $(python3 tests/nuc980_ebi.py check $D/dump.jsonl | head -1)" >> $D/progress.log
done
python3 tests/nuc980_ebi.py check $D/dump.jsonl > $D/final.txt
date > $D/finished
