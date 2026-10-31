#!/usr/bin/env python3
"""Exercise the service stop guard with the real serial class on a PTY only."""
import os, pty, select, signal, subprocess, sys, tempfile
from pathlib import Path
binary = Path(sys.argv[1]).resolve()
for mode, sig, expected in [
    ('service', signal.SIGTERM, b'gd\r\n'),
    ('service', signal.SIGINT, b'gd\r\n'),
    ('unwind', signal.SIGTERM, b'gd\r\n'),
    ('manual', signal.SIGTERM, b''),
    ('normal', None, b''),
    ('error', None, b'')]:
    master, slave = pty.openpty()
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='gimbal-stop-test-') as folder:
            config = Path(folder) / 'test.yaml'
            config.write_text(f'com_port: {os.ttyname(slave)}\nbaudrate: 115200\n')
            process = subprocess.Popen([str(binary), str(config), mode], stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            assert select.select([process.stdout], [], [], 5)[0], 'client startup timed out'
            assert process.stdout.readline().strip() == 'READY', process.communicate(timeout=3)
            if sig is not None:
                process.send_signal(sig)
            process.communicate(timeout=5)
            data = b''
            while select.select([master], [], [], .05)[0]:
                data += os.read(master, 1024)
            assert data == expected, (mode, sig, data)
            print('PASS signal stop:', mode, sig, repr(data))
    finally:
        if process is not None and process.poll() is None:
            process.kill(); process.communicate()
        os.close(master); os.close(slave)
