# 解析に使った道具

作業用 PC の `~/vm` で、LA Viewer（XP の仮想マシン）の通信を調べたときのスクリプトです。パスは作業用 PC のもの（`/home/user/vm/qmp.sock` など）のままです。

- `vm/qmp.py`、`click.py`、`drag.py`、`keys.py`: QEMU の QMP で、仮想マシンの画面を撮り、クリック、ドラッグ、キー入力をします。
- `vm/trig_sweep.py`: トリガの設定を順に変えて、LA Viewer に Run をさせます。
- `vm/volt_step.sh`: LA Viewer のしきい値のダイアログを操作します（座標は 1024×768 の画面のもの）。
- `cap/usbdump.py`: usbmon の pcap を、命令ごとの文字に直します。
- `cap/volt_an.py`、`08_times.txt`: しきい値（命令 21）を解いたときの解析と、操作の時刻です。
- `atom/put.py`: AtomS3U（MicroPython）に、試験用の信号（CH0〜3 に 1kHz、10kHz、100kHz、1MHz）を出す `main.py` を書き込みます。

記録（pcap とその書き起こし）、LA Viewer とドライバ、FPGA の回路データ、AtomS3U の元のファームウェアのバックアップは、メーカーのものなので入れません。作業用 PC の `~/vm` にあります。
