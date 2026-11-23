import serial,time,sys
main = """from machine import Pin, PWM
# LA CH0..3 = GPIO14, 17, 42, 40: 1 kHz, 10 kHz, 100 kHz, 1 MHz square waves
pw = [PWM(Pin(p), freq=f, duty_u16=32768) for p, f in ((14, 1000), (17, 10000), (42, 100000), (40, 1000000))]
"""
s=serial.Serial("/dev/ttyACM0",115200,timeout=1)
s.write(b"\r\x03\x03"); time.sleep(0.5); s.read(5000)
s.write(b"\x01"); time.sleep(0.3); s.read(5000)   # raw REPL
code = "f=open(\"main.py\",\"w\");f.write(%r);f.close()\nexec(open(\"main.py\").read())\nprint(\"ok\",[p.freq() for p in pw])\n" % main
s.write(code.encode()+b"\x04"); time.sleep(1.5)
print(s.read(5000))
s.write(b"\x02")
