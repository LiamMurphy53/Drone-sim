#!/usr/bin/env python3
"""Exercise the actual patched SITL motor functions with a native C harness."""
import subprocess, tempfile, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

class MotorFrameTests(unittest.TestCase):
    def test_motor_frames_remain_complete_across_sensor_gate(self):
        source = (ROOT/'vendor/betaflight-4.5.2/src/main/target/SITL/sitl.c').read_text()
        functions = '\n'.join(function(source, s) for s in [
            'static void pwmWriteMotor(uint8_t index, float value)',
            'static void pwmCompleteMotorUpdate(void)'])
        harness = r'''
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <stddef.h>
#include <stdio.h>
#define MAX_SUPPORTED_MOTORS 4
#define FEATURE_3D 1
typedef struct {float motor_speed[4];} servo_packet;
typedef struct {uint16_t motorCount; float pwm_output_raw[16];} servo_packet_raw;
static pthread_mutex_t updateLock = PTHREAD_MUTEX_INITIALIZER;
static bool motorUpdatePending;
static int16_t motorsPwm[4], idlePulse=1000;
static servo_packet pwmPkt;
static servo_packet_raw pwmRawPkt = {.motorCount=4}, received;
static int pwmLink, pwmRawLink, rawCount;
static bool featureIsEnabled(int feature) {(void)feature; return false;}
static int udpSend(void *link, const void *data, size_t size) {
    (void)size;
    if (link == &pwmRawLink) {received=*(const servo_packet_raw *)data; ++rawCount;}
    return 0;
}
'''+functions+r'''
int main(void) {
    for (int frame=0; frame<100; ++frame) {
        // The sensor gate is closed while the FC computes new motor values.
        // A gate must delay publication, never discard individual writes.
        pthread_mutex_lock(&updateLock);
        for (int motor=0; motor<4; ++motor)
            pwmWriteMotor(motor, 1100 + frame*4 + motor);
        motorUpdatePending=true;
        pthread_mutex_unlock(&updateLock);
        pwmCompleteMotorUpdate();
        if (rawCount != frame+1) return 1;
        for (int motor=0; motor<4; ++motor)
            if (received.pwm_output_raw[motor] != 1100 + frame*4 + motor) return 2;
        // No sensor update: no duplicate publication.
        pwmCompleteMotorUpdate();
        if (rawCount != frame+1) return 3;
    }
    puts("100 complete motor frames; no skipped motor writes or duplicate frames");
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='drone-motor-frame-') as directory:
            path = Path(directory)
            (path/'test.c').write_text(harness)
            subprocess.run(['clang','-std=c11','-Wall','-Wextra',str(path/'test.c'),
                            '-lpthread','-o',str(path/'test')],check=True,capture_output=True,text=True)
            result = subprocess.run([str(path/'test')],capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)

if __name__ == '__main__':
    unittest.main()
