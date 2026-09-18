#!/usr/bin/env python3
"""Configure only our private SITL instance. Never connects to a physical FC."""
import socket, subprocess, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT/'vendor/betaflight-4.5.2/obj/main/betaflight_SITL.elf'
RUNTIME = ROOT/'runtime'
CONFIG_MARKER = RUNTIME/'configured-v3'

def configure():
    RUNTIME.mkdir(exist_ok=True)
    # Fail before starting if another service owns the private simulator ports.
    for port, kind in [(5761,socket.SOCK_STREAM),(9003,socket.SOCK_DGRAM),(9004,socket.SOCK_DGRAM)]:
        with socket.socket(socket.AF_INET,kind) as check: check.bind(('127.0.0.1',port))
    with (RUNTIME/'configure.log').open('w') as log:
        proc = subprocess.Popen([str(BIN)],cwd=RUNTIME,stdout=log,stderr=log)
        # CLI setup runs without sensor input; the live simulator feeds sensors after restart.
        try:
            conn=None
            for _ in range(100):
                if proc.poll() is not None: raise RuntimeError(f'Betaflight exited ({proc.returncode}); see runtime/configure.log')
                try: conn=socket.create_connection(('127.0.0.1',5761),timeout=.2); break
                except OSError: time.sleep(.05)
            if conn is None: raise RuntimeError('Betaflight TCP port did not open')
            with conn:
                conn.settimeout(.15)
                def read():
                    data=b''
                    while True:
                        try:
                            chunk=conn.recv(65536)
                            if not chunk: break
                            data+=chunk
                        except socket.timeout: break
                    return data.decode(errors='replace')
                conn.sendall(b'#'); time.sleep(.2); transcript=read()
                commands = [
                    'set craft_name = Deadcat Lab', 'mixer QUADX', 'map AETR1234',
                    'feature -GPS', 'feature -TELEMETRY',
                    'feature AIRMODE', 'feature -MOTOR_STOP', 'feature ANTI_GRAVITY',
                    'set motor_pwm_protocol = PWM', 'set motor_pwm_rate = 480',
                    'set min_command = 1000', 'set min_throttle = 1000', 'set max_throttle = 2000',
                    'set pid_process_denom = 1', 'set gyro_calib_duration = 50',
                    'set small_angle = 180', 'set yaw_motors_reversed = OFF', 'set yaw_control_reversed = OFF',
                    'set acc_hardware = AUTO', 'set acc_calibration = 0,0,0,1',
                    'set gyro_lpf1_static_hz = 80', 'set gyro_lpf2_static_hz = 100',
                    'set dterm_lpf1_static_hz = 50',
                    'set dterm_lpf2_static_hz = 70',
                    # Gentle initial tune for the inherited 50 ms motor lag.
                    'set p_roll = 8', 'set i_roll = 8', 'set d_roll = 0', 'set f_roll = 0',
                    'set p_pitch = 8', 'set i_pitch = 8', 'set d_pitch = 0', 'set f_pitch = 0',
                    'set p_yaw = 20', 'set i_yaw = 3', 'set f_yaw = 0',
                    # Anti-gravity adds integral gain independently of i_roll /
                    # i_pitch. Scale the stock 80 boost with our 8-vs-80 I tune;
                    # otherwise throttle chops briefly restore a much hotter tune.
                    'set anti_gravity_gain = 8', 'set anti_gravity_p_gain = 100',
                    'set rates_type = BETAFLIGHT', 'set roll_rc_rate = 80', 'set pitch_rc_rate = 80',
                    'set yaw_rc_rate = 70', 'set roll_srate = 60', 'set pitch_srate = 60', 'set yaw_srate = 50',
                    'set roll_expo = 20', 'set pitch_expo = 20', 'set yaw_expo = 15',
                    'aux 0 0 0 1700 2100 0 0', 'save']
                for cmd in commands:
                    conn.sendall((cmd+'\n').encode()); time.sleep(.035)
                    response=read(); transcript+=response
                    if 'ERROR' in response or 'Invalid' in response:
                        raise RuntimeError('SITL rejected '+cmd+': '+response)
                (RUNTIME/'configuration.txt').write_text(transcript)
            CONFIG_MARKER.write_text('Betaflight 4.5.2; scaled throttle-transient PID boost; local simulation only\n')
        finally:
            if proc.poll() is None:
                proc.terminate()
                try: proc.wait(3)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait()
if __name__ == '__main__':
    configure(); print('Betaflight configured for the simulator.')
