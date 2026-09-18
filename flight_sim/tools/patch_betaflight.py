#!/usr/bin/env python3
"""Idempotent transport/time fixes for real-time Godot on macOS; no PID changes."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]
p=root/'vendor/betaflight-4.5.2/src/main/target/SITL/sitl.c'
s=p.read_text()
if 'FLIGHT_LAB_REALTIME' not in s:
    s=s.replace('simRate = deltaSim / (out_ts.tv_sec + 1e-9*out_ts.tv_nsec);',
        'simRate = 1.0; // FLIGHT_LAB_REALTIME: frame-batched UDP must not accelerate the FC clock.')
    s=re.sub(r'uint64_t micros64\(void\)\n\{.*?\n\}',
        'uint64_t micros64(void)\n{\n    return micros64_real();\n}',s,flags=re.S)
    s=re.sub(r'uint64_t millis64\(void\)\n\{.*?\n\}',
        'uint64_t millis64(void)\n{\n    return millis64_real();\n}',s,flags=re.S)
    s=s.replace('virtualAccSet(virtualAccDev, x, y, z);','if (virtualAccDev) virtualAccSet(virtualAccDev, x, y, z);')
    s=s.replace('virtualGyroSet(virtualGyroDev, x, y, z);','if (virtualGyroDev) virtualGyroSet(virtualGyroDev, x, y, z);')
    p.write_text(s)
    print('Applied real-time clock and sensor initialization guards.')
# The upstream TCP worker can destroy a newly allocated stream before the main
# thread finishes opening it. Serialize Dyad operations and its write buffers.
s=p.read_text()
if 'flightLabTcpLock' not in s:
    s=s.replace('static void* tcpThread(void* data)', 'extern pthread_mutex_t flightLabTcpLock;\n\nstatic void* tcpThread(void* data)')
    s=s.replace('    dyad_init();', '    pthread_mutex_lock(&flightLabTcpLock);\n    dyad_init();')
    s=s.replace('    dyad_setUpdateTimeout(0.01f);', '    dyad_setUpdateTimeout(0.0f);\n    pthread_mutex_unlock(&flightLabTcpLock);')
    s=s.replace('        dyad_update();', '        pthread_mutex_lock(&flightLabTcpLock);\n        dyad_update();\n        pthread_mutex_unlock(&flightLabTcpLock);\n        usleep(1000);')
    s=s.replace('    dyad_shutdown();', '    pthread_mutex_lock(&flightLabTcpLock);\n    dyad_shutdown();\n    pthread_mutex_unlock(&flightLabTcpLock);')
    p.write_text(s)
p=root/'vendor/betaflight-4.5.2/src/main/drivers/serial_tcp.c'
s=p.read_text()
if 'flightLabTcpLock' not in s:
    s=s.replace('#define BASE_PORT 5760', '#define BASE_PORT 5760\n\npthread_mutex_t flightLabTcpLock = PTHREAD_MUTEX_INITIALIZER;')
    s=s.replace('    s->serv = dyad_newStream();', '    pthread_mutex_lock(&flightLabTcpLock);\n    s->serv = dyad_newStream();')
    s=s.replace('    return s;\n}\n\nserialPort_t *serTcpOpen', '    pthread_mutex_unlock(&flightLabTcpLock);\n    return s;\n}\n\nserialPort_t *serTcpOpen')
    s=s.replace('    if (s->conn == NULL) return;', '    pthread_mutex_lock(&flightLabTcpLock);\n    if (s->conn == NULL) {\n        pthread_mutex_unlock(&flightLabTcpLock);\n        return;\n    }')
    s=s.replace('    pthread_mutex_unlock(&s->txLock);\n}\n\nvoid tcpDataIn', '    pthread_mutex_unlock(&s->txLock);\n    pthread_mutex_unlock(&flightLabTcpLock);\n}\n\nvoid tcpDataIn')
    # Simulation services should not be exposed to other machines.
    s=s.replace('dyad_listenEx(s->serv, NULL,', 'dyad_listenEx(s->serv, "127.0.0.1",')
    p.write_text(s)
p=root/'vendor/betaflight-4.5.2/src/main/target/SITL/sitl.c'
s=p.read_text()
if '#include <unistd.h>' not in s:
    p.write_text(s.replace('#include <time.h>', '#include <time.h>\n#include <unistd.h>'))
