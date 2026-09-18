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

# Publish complete motor frames. Upstream uses a mutex as a cross-thread
# semaphore and skips individual motor writes while it is held. A state packet
# arriving partway through the four writes can therefore publish a mixed frame.
s=p.read_text()
if 'FLIGHT_LAB_COMPLETE_MOTOR_FRAME' not in s:
    replacements = [
        ('static pthread_mutex_t updateLock;',
         'static pthread_mutex_t updateLock;\nstatic bool motorUpdatePending = false; // FLIGHT_LAB_COMPLETE_MOTOR_FRAME'),
        ('    pthread_mutex_unlock(&updateLock); // can send PWM output now\n\n#if defined(SIMULATOR_GYROPID_SYNC)',
         '    pthread_mutex_lock(&updateLock);\n    motorUpdatePending = true;\n    pthread_mutex_unlock(&updateLock);\n\n#if defined(SIMULATOR_GYROPID_SYNC)'),
        ('    if (pthread_mutex_trylock(&updateLock) != 0) return;\n\n    if (index < MAX_SUPPORTED_MOTORS)',
         '    // Only the FC thread writes this frame; always update every motor.\n    if (index < MAX_SUPPORTED_MOTORS)'),
        ('    pthread_mutex_unlock(&updateLock); // can send PWM output now\n}\n\nstatic void pwmWriteMotorInt',
         '}\n\nstatic void pwmWriteMotorInt'),
        ('    // get one "fdm_packet" can only send one "servo_packet"!!\n    if (pthread_mutex_trylock(&updateLock) != 0) return;',
         '    // Consume one pending sensor update after all motor writes finish.\n'
         '    pthread_mutex_lock(&updateLock);\n'
         '    const bool sendFrame = motorUpdatePending;\n'
         '    motorUpdatePending = false;\n'
         '    pthread_mutex_unlock(&updateLock);\n'
         '    if (!sendFrame) return;')]
    for before,after in replacements:
        if s.count(before) != 1:
            raise RuntimeError('Unexpected SITL source while patching complete motor frames')
        s=s.replace(before,after)
    p.write_text(s)
    print('Patched coherent motor-frame publication and cross-thread locking.')
