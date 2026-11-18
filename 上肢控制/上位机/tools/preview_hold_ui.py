"""Read-only, synthetic browser preview. No serial import or CAN transport.

All POSTs are rejected. Never use this preview's telemetry as motor feedback.
"""
from pathlib import Path
import json
import math
import sys
import time
import argparse
import shutil
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
from classic_dual_controller import ClassicConfigStore, DualMotorController, MotionPlan
from dataset_recorder import DatasetRecorder


class NoHardware:
    def send(self, *_args):
        raise RuntimeError("Offline UI preview cannot send CAN frames")


controller = DualMotorController(NoHardware(), ClassicConfigStore(APP / 'classic_dual_config.json').load())
motion_demo = False
recorder = None
for key, runtime in controller.runtimes.items():
    cfg = controller.config['joints'][key]
    runtime.verified = True
    runtime.observed_uid = cfg['expected_uid']
    runtime.actual_deg = runtime.target_deg = runtime.setpoint_deg = 30.0 if key == 'shoulder' else 45.0
    runtime.actual_raw_rad = cfg['software_zero_rad'] + cfg['direction']*math.radians(runtime.actual_deg)
    runtime.phase = 'controlling'
    runtime.on_target = True
    runtime.mode_status = 2
    runtime.temperature_c = 25.0
    runtime.parameter_values = {0x7005:0,0x700B:3.5,0x7018:6.5,0x701A:1.0,0x701C:24.0}
controller.motion_plan = MotionPlan(0,time.monotonic(),1.0,('elbow','shoulder'),
    {'elbow':45.,'shoulder':30.},{'elbow':45.,'shoulder':30.},
    {'elbow':3.5,'shoulder':3.5},{'elbow':3.,'shoulder':3.},{'elbow':1.,'shoulder':1.})


def enable_diagnostic_demo():
    """Synthetic display values only; NoHardware and POST rejection stay in force."""
    for key, runtime in controller.runtimes.items():
        sign = 1 if key == 'shoulder' else -1
        runtime.torque_nm = sign * 1.6
        runtime.gravity_torque_nm = sign * .4
        runtime.load_bias_nm = sign * .8
        runtime.last_sent_control = {
            'sent_at':time.monotonic(), 'kp':12., 'kd':1.5,
            'feedforward_nm':sign*1.2, 'pd_estimate_nm':sign*.5,
            'estimated_total_nm':sign*1.7, 'torque_limit_nm':3.5,
        }


class PreviewHandler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,directory=str(APP/'web'),**kwargs)

    def do_GET(self):
        if self.path.startswith('/api/dataset/download/') and recorder is not None:
            try:
                archive = recorder.archive(self.path.removeprefix('/api/dataset/download/'))
                self.send_response(200)
                self.send_header('Content-Type','application/zip')
                self.send_header('Content-Disposition',f'attachment; filename="SYNTHETIC_ONLY_{archive.name}"')
                self.send_header('Content-Length',str(archive.stat().st_size))
                self.end_headers()
                with archive.open('rb') as handle:
                    shutil.copyfileobj(handle,self.wfile)
            except (ValueError,RuntimeError,OSError) as exc:
                self.send_error(409,str(exc))
        elif self.path.startswith('/api/state'):
            for key, runtime in controller.runtimes.items():
                if motion_demo:
                    runtime.actual_deg = (30. if key == 'shoulder' else 45.) + 10*math.sin(time.monotonic())
                    cfg = controller.config['joints'][key]
                    runtime.actual_raw_rad = cfg['software_zero_rad'] + cfg['direction']*math.radians(runtime.actual_deg)
                runtime.last_feedback_at = time.monotonic()
                runtime.parameter_times = {i:time.monotonic() for i in runtime.parameter_values}
                if runtime.last_sent_control is not None:
                    runtime.last_sent_control['sent_at'] = time.monotonic()
                if recorder is not None:
                    recorder.append('feedback', {'joint':key, 'motor_can_id':controller.config['joints'][key]['motor_can_id'],
                        'identity_verified':True, 'position_joint_deg':runtime.actual_deg,
                        'position_joint_rad':math.radians(runtime.actual_deg), 'temperature_c':25., 'mode_status':0})
            state = {**controller.snapshot(),'version':'2.3.3','connected':True,'port':'OFFLINE',
                     'ports':[], 'logs':[], 'dataset':recorder.status() if recorder else {'supported':False},
                     'status':'离线合成界面测试，非实机反馈；电机控制接口全部禁用'}
            self.json_response(state)
        elif self.path.startswith('/api/history'):
            history = {'elbow':{},'shoulder':{}}
            if any(runtime.last_sent_control is not None for runtime in controller.runtimes.values()):
                now = time.time()
                for key, runtime in controller.runtimes.items():
                    sign = 1 if key == 'shoulder' else -1
                    for metric in ('angle','velocity','torque','temperature'):
                        samples = []
                        for index in range(121):
                            stamp = now - 60 + index*.5
                            wave = math.sin(index*.12)
                            value = {'angle':runtime.actual_deg + wave*.5,
                                     'velocity':wave*.02, 'torque':sign*(1.6+wave*.2),
                                     'temperature':25+wave*.1}[metric]
                            samples.append([stamp,value])
                        history[key][metric] = samples
            self.json_response(history)
        elif self.path in ('/','/index.html'):
            body = (APP/'web/index.html').read_text(encoding='utf-8').replace('<strong>CYBERARM</strong>','<strong>离线 UI 测试</strong>').encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type','text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(body)
        else:
            super().do_GET()

    def json_response(self,value):
        body = json.dumps(value,ensure_ascii=False).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type','application/json; charset=utf-8')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if recorder is not None and self.path in ('/api/dataset/start','/api/dataset/finish'):
            try:
                size = int(self.headers.get('Content-Length','0'))
                body = json.loads(self.rfile.read(size) or b'{}')
                if body:
                    raise ValueError('No parameters allowed')
                if self.path.endswith('/start'):
                    result = recorder.start(controller.config,interface='offline_synthetic')
                else:
                    result = recorder.finish('user')
                self.json_response({'ok':True,'result':result,'state':{**controller.snapshot(),
                    'version':'2.3.3','connected':True,'port':'OFFLINE','ports':[], 'logs':[],
                    'dataset':recorder.status(),'status':'离线合成界面测试，非实机反馈；电机控制接口全部禁用'}})
            except (ValueError,RuntimeError) as exc:
                self.send_error(409,str(exc))
            return
        self.send_error(403,'Offline preview: all controls disabled')

    def log_message(self,*_args):
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stopped', action='store_true', help='Preview editable settings without hardware')
    parser.add_argument('--diagnostic-demo', action='store_true', help='Show clearly synthetic signed torque bars')
    parser.add_argument('--motion-demo', action='store_true', help='Animate synthetic actual feedback without any motor commands')
    parser.add_argument('--dataset-demo', action='store_true', help='Allow synthetic dataset recording only; all motor POSTs remain forbidden')
    args = parser.parse_args()
    motion_demo = args.motion_demo
    if args.dataset_demo:
        recorder = DatasetRecorder(APP/'tmp'/'dataset_preview',source_root=APP)
    if args.diagnostic_demo:
        enable_diagnostic_demo()
    if args.stopped:
        controller.motion_plan = None
        for runtime in controller.runtimes.values():
            runtime.phase = 'stopped'
            runtime.mode_status = 0
    server = ThreadingHTTPServer(('127.0.0.1',18765),PreviewHandler)
    print('OFFLINE_ONLY http://127.0.0.1:18765/ ; no CAN/serial; POST disabled',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if recorder is not None:
            recorder.finish('offline_preview_closed')
            recorder.wait()
        server.server_close()
