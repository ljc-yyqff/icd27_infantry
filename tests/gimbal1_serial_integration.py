import os
import select
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
BUILD = Path(sys.argv[1]).resolve()

class Board:

    def __init__(self, mode='normal', pitch=25):
        (self.master, self.slave) = os.openpty()
        self.port = os.ttyname(self.slave)
        (self.mode, self.pitch) = (mode, pitch)
        self.axis_mode = 0 if mode == 'encoder' else 1
        self.yaw = 0.0
        (self.ytarget, self.ptarget) = (self.yaw, self.pitch)
        self.lines = []
        self.enabled_at = None
        self.stop = False
        self.buffer = b''
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, line):
        data = (line + '\r\n').encode()
        os.write(self.master, data[:3])
        time.sleep(max(0.003, len(data) / 11520) if self.mode in ('long', 'disabled') else 0.003)
        os.write(self.master, data[3:])

    def run(self):
        next_telemetry = time.monotonic() + 0.1
        while not self.stop:
            now = time.monotonic()
            if now >= next_telemetry:
                if not (self.mode in ('drop', 'noreplies_drop') and self.enabled_at and (now - self.enabled_at > 0.8)):
                    if self.mode in ('nan', 'noreplies_nan'):
                        self.send('nan,nan,nan,nan,0,0,20,0.1,-0.2')
                    else:
                        self.send(f"{self.ptarget:.3f},{self.pitch - (0.087 if self.mode == 'noreplies_offset' else 0):.3f},{self.ytarget:.3f},{self.yaw:.3f},0,0,20,0.1,-0.2")
                next_telemetry = now + 0.02
            if not select.select([self.master], [], [], 0.002)[0]:
                continue
            self.buffer += os.read(self.master, 4096)
            while b'\n' in self.buffer:
                (raw, self.buffer) = self.buffer.split(b'\n', 1)
                line = raw.decode(errors='replace').strip()
                if not line:
                    continue
                self.lines.append((time.monotonic(), line))
                if line == 'gmode status':
                    if self.mode in ('missinggm', 'noreplies', 'noreplies_stuck', 'noreplies_drop', 'noreplies_nan'):
                        continue
                    active = self.enabled_at is not None
                    ys = ps = 4 if active else 0
                    yf = pf = 0
                    if self.mode == 'notready' and active:
                        ps = 2
                    if self.mode == 'disabled':
                        ys = ps = 0
                    if self.mode == 'fault' and active and (now - self.enabled_at > 0.9):
                        yf = 3
                        ys = 5
                    valid = 0 if self.mode == 'imuwarming' else 1
                    self.send(f'#GM,id=1,ok=1,reason=accepted,ys={ys},yf={yf},ym={self.axis_mode},yrq={self.axis_mode},ps={ps},pf={pf},pm={self.axis_mode},prq={self.axis_mode},yg=0,pg=0,ya=0,pa=0,pe={self.pitch},lo=0,hi=41,sg=-1,valid={valid},age=1,sa=4294967295')
                elif line == 'ge':
                    self.enabled_at = time.monotonic()
                    (self.ytarget, self.ptarget) = (self.yaw, self.pitch)
                    ok = 0 if self.mode == 'gereject' else 1
                    reason = 'imu_not_ready' if not ok else 'preparing'
                    if self.mode.startswith('noreplies'):
                        continue
                    self.send(f'#GC,cmd=ge,ok={ok},reason={reason},state={(1 if ok else 0)},mode={self.axis_mode},target_deg={self.yaw:.3f}')
                elif line.startswith('yr '):
                    delta = float(line.split()[1])
                    ok = self.mode != 'rejectmove'
                    if ok:
                        self.ytarget += delta
                        if self.mode not in ('stuck', 'noreplies_stuck'):
                            self.yaw = self.ytarget
                    if self.mode != 'dropgc' and (not self.mode.startswith('noreplies')):
                        self.send(f"#GC,cmd=yr,ok={(1 if ok else 0)},reason={('accepted' if ok else 'mechanical_zero_not_ready')},state=4,mode={self.axis_mode},target_deg={self.ytarget:.3f}")
                elif line.startswith('pr '):
                    delta = float(line.split()[1])
                    ok = self.mode != 'rejectmove'
                    if ok:
                        self.ptarget += delta
                        if self.mode not in ('stuck', 'noreplies_stuck'):
                            self.pitch = self.ptarget
                    if self.mode != 'dropgc' and (not self.mode.startswith('noreplies')):
                        self.send(f"#GC,cmd=pr,ok={(1 if ok else 0)},reason={('accepted' if ok else 'mechanical_zero_not_ready')},state=4,mode={self.axis_mode},target_deg={self.ptarget:.3f}")

    def close(self):
        self.stop = True
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)

def run_case(name, actions, mode='normal', pitch=25, exe='gimbal1_test', extra=(), require_status=True):
    board = Board(mode, pitch)
    try:
        with tempfile.TemporaryDirectory(prefix='gimbal1-test-') as folder:
            config = Path(folder) / 'config.yaml'
            config.write_text(f'com_port: {board.port}\nbaudrate: 115200\ngimbal1_max_relative_angle_deg: 10\ngimbal1_require_status_reply: {str(require_status).lower()}\nserial_offline_timeout_ms: 100\ngimbal1_enable_timeout_s: 1.0\ngimbal1_imu_ready_timeout_s: 1.5\ngimbal1_command_receipt_timeout_ms: 250\ngimbal1_sweep:\n  pitch_center_deg: 25\n  pitch_min_deg: 5\n  pitch_max_deg: 40\n')
            process = subprocess.Popen([str(BUILD / exe), str(config), *extra], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                for (delay, command) in actions:
                    time.sleep(delay)
                    if process.poll() is None:
                        process.stdin.write(command + '\n')
                        process.stdin.flush()
                (output, _) = process.communicate(timeout=8)
            except BaseException:
                process.kill()
                process.communicate()
                raise
        lines = [line for (_, line) in board.lines]
        moves = [line for line in lines if line.startswith(('yr ', 'pr '))]
        for ((a, _), (b, _)) in zip(board.lines, board.lines[1:]):
            assert b - a >= 0.018, (name, 'outbound rate exceeded', b - a)
        status_times = [t for (t, line) in board.lines if line == 'gmode status']
        assert all((b - a >= 0.018 for (a, b) in zip(status_times, status_times[1:]))), (name, 'status rate too high')
        assert all((len(line) <= 31 for line in lines)), (name, 'oversize line')
        return (output, moves, lines, process.returncode)
    finally:
        board.close()
for mode in ('noreplies_stuck', 'noreplies_drop', 'noreplies_nan', 'missinggm', 'fault'):
    out,moves,lines,code=run_case(mode,[(0,'enable'),(0.3,'p 10 10'),(3.5,'p 10 10'),(0.3,'status'),(0.1,'quit')],mode,require_status=False)
    assert code == 0 and len(moves)==2,(mode,out,lines)
    assert lines.count('gd')==1,(mode,lines)
    assert not any(q in lines for q in ('gmode status','yprobe 1','pprobe')),(mode,lines)
    assert '[interlock]' not in out,(mode,out)
    print('PASS protocol-only repeated relative commands without arrival/feedback/state gate:',mode)
out,moves,lines,code=run_case('EOF',[(0,'enable'),(0.3,'p 1 5')],'noreplies',require_status=False)
assert code==0 and len(moves)==1 and 'gd' not in lines,(out,lines)
print('PASS EOF closes serial without automatic gd')

out,moves,lines,code=run_case('board ge rejection is visible',[(0,'enable'),(0.5,'quit')],'gereject',require_status=False)
assert code==0 and '[board RX] cmd=ge ok=0 reason=imu_not_ready' in out,(out,lines)
assert lines.count('gd')==1,(out,lines)
print('PASS rejected ge is shown verbatim without automatic motor action')
