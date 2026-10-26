"""Explicit GPU/NPU inference in isolated host environments, without rover controls."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
import uuid
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import numpy as np
from rover_training.geometry import FOUR_CLASS_NAMES
from .detectors import DetectionResult


def validate_remote_model(status, model_path):
    metadata = status['model']
    backend = metadata.get('backend')
    requested = metadata.get('source_checkpoint') if backend == 'npu' else metadata.get('model')
    digest = metadata.get('model_sha256', '')
    if (not status.get('ready') or requested != model_path or
            metadata.get('class_names') != list(FOUR_CLASS_NAMES) or
            backend not in ('torch', 'npu') or not isinstance(digest, str) or len(digest) != 64 or
            any(c not in '0123456789abcdef' for c in digest)):
        raise ValueError('Detector service does not match the requested trained rover model')
    if backend == 'npu':
        source_digest = metadata.get('source_checkpoint_sha256', '')
        if (metadata.get('proof_state') != 'npu_executed' or
                metadata.get('computation_offload_verified') is not True or
                metadata.get('compile_evidence', {}).get('verified') is not True or
                metadata.get('model_family') != 'YOLOv8' or
                not isinstance(source_digest, str) or len(source_digest) != 64 or
                any(c not in '0123456789abcdef' for c in source_digest)):
            raise ValueError('NPU detector requires verified YOLOv8 execution and compilation evidence')
    return metadata


def validate_rows(rows, width, height):
    import math
    if not isinstance(rows, list) or len(rows) > 300:
        raise ValueError('Invalid detection list')
    for row in rows:
        index = row['class_id']
        if (isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(FOUR_CLASS_NAMES) or
                row['class_name'] != FOUR_CLASS_NAMES[index]):
            raise ValueError('Unexpected rover class')
        values = [row['confidence'], *row['xyxy']]
        if len(values) != 5 or not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
            raise ValueError('Invalid detection coordinates/confidence')
        x1, y1, x2, y2 = row['xyxy']
        if not 0 <= row['confidence'] <= 1 or not 0 <= x1 < x2 <= width or not 0 <= y1 < y2 <= height:
            raise ValueError('Detection outside image')
    return rows


class RemoteDetector:
    def __init__(self, endpoint, model_path):
        parsed = urlsplit(endpoint)
        if parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost') or parsed.path not in ('', '/'):
            raise ValueError('Detector service must be a loopback HTTP endpoint')
        self.endpoint = endpoint.rstrip('/')
        with urlopen(self.endpoint+'/status', timeout=2.) as response:
            status = json.loads(response.read(65536))
        self.metadata = validate_remote_model(status, model_path)

    def detect(self, image, conf=.25):
        height, width = image.shape[:2]
        request = Request(self.endpoint+'/predict', data=image.tobytes(), method='POST', headers={
            'Content-Type': 'application/octet-stream', 'X-Image-Width': str(width), 'X-Image-Height': str(height),
            'X-Confidence': str(conf), 'X-Model-Sha256': self.metadata['model_sha256']})
        with urlopen(request, timeout=2.) as response:
            data = json.loads(response.read(1024*1024))
        if (data['model_sha256'] != self.metadata['model_sha256'] or
                data.get('backend', self.metadata['backend']) != self.metadata['backend']):
            raise ValueError('Detector model changed during the session')
        return DetectionResult(validate_rows(data['detections'], width, height), data['timings'])


def service_handler(detector):
    inference_lock = threading.Lock()
    state_lock = threading.Lock()
    counters = dict(processed_frames=0, last_processing_ms=None, error=None,
                    processing_session_id=str(uuid.uuid4()))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, data):
            body = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def local(self):
            return (not self.headers.get('Origin') and
                    self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'))

        def do_GET(self):
            if not self.local():
                return self.reply(403, dict(error='Loopback detector clients only'))
            if self.path == '/status':
                with state_lock:
                    status = dict(ready=counters['error'] is None, model=detector.metadata, **counters)
                return self.reply(200, status)
            self.reply(404, dict(error='Not found'))

        def do_POST(self):
            if not self.local():
                return self.reply(403, dict(error='Loopback detector clients only'))
            if self.path != '/predict':
                return self.reply(404, dict(error='Not found'))
            acquired = False
            try:
                width, height = int(self.headers['X-Image-Width']), int(self.headers['X-Image-Height'])
                length = int(self.headers.get('Content-Length', '0'))
                if (not 1 <= width <= 1920 or not 1 <= height <= 1080 or length != width*height*3 or
                        self.headers.get('Content-Type') != 'application/octet-stream' or
                        self.headers.get('X-Model-Sha256') != detector.metadata['model_sha256']):
                    raise ValueError('Invalid dimensions, payload or model hash')
                confidence = float(self.headers['X-Confidence'])
                if not 0 <= confidence <= 1:
                    raise ValueError('Invalid confidence')
                self.connection.settimeout(2.)
                payload = self.rfile.read(length)
                if len(payload) != length:
                    raise ValueError('Truncated image')
                image = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, 3)
                acquired = inference_lock.acquire(blocking=False)
                if not acquired:
                    return self.reply(429, dict(error='One inference at a time; discard old work'))
                with state_lock:
                    error = counters['error']
                if error:
                    return self.reply(503, dict(error=error))
                started = time.monotonic()
                try:
                    result = detector.detect(image, confidence)
                    validate_rows(result.detections, width, height)
                except Exception as error:
                    with state_lock:
                        counters['error'] = f'{type(error).__name__}: {error}; restart the selected backend'
                    return self.reply(503, dict(error=counters['error']))
                with state_lock:
                    counters['processed_frames'] += 1
                    counters['last_processing_ms'] = (time.monotonic()-started)*1000
                self.reply(200, dict(detections=result.detections, timings=result.timings,
                                     backend=detector.metadata['backend'],
                                     model_sha256=detector.metadata['model_sha256']))
            except (ValueError, TypeError, KeyError, TimeoutError) as error:
                self.reply(400, dict(error=str(error)))
            except Exception as error:
                self.reply(503, dict(error=str(error)))
            finally:
                if acquired:
                    inference_lock.release()
    return Handler


def prepare_detector(model, backend='torch', model_manifest=None, device='0'):
    from pathlib import Path
    from .detectors import OnnxDetector, TorchDetector, sha256_file, validate_manifest
    checkpoint = str(Path(model).resolve(strict=True))
    if backend == 'npu':
        if not model_manifest:
            raise ValueError('NPU mode requires --model-manifest; no automatic GPU/CPU fallback')
        manifest = validate_manifest(model_manifest)
        if (manifest['source_checkpoint_sha256'] != sha256_file(checkpoint) or
                manifest['class_names'] != list(FOUR_CLASS_NAMES) or
                manifest['output_format'] != 'yolo_dfl'):
            raise ValueError('Use the verified four-class YOLOv8 DFL deployment for this checkpoint')
        detector = OnnxDetector(model_manifest, backend='npu')
    elif backend == 'torch':
        import torch
        if model_manifest:
            raise ValueError('--model-manifest belongs to the NPU backend')
        if not torch.cuda.is_available() or device not in ('0', 'cuda:0'):
            raise RuntimeError('GPU mode requires the existing GPU runtime; no automatic fallback')
        torch.set_num_threads(1)
        detector = TorchDetector(model_path=checkpoint, device=device, imgsz=960)
    else:
        raise ValueError('Choose the torch or npu backend explicitly')
    if list(detector.names.values()) != list(FOUR_CLASS_NAMES):
        raise ValueError('Use the reviewed four-class rover checkpoint')
    for _ in range(3):
        detector.warmup()
    detector.metadata['source_checkpoint'] = checkpoint
    if backend == 'npu':
        from .npu_tools import detector_metadata
        detector.metadata = detector_metadata(detector)
        detector.metadata.update(executed_device='npu', model_family='YOLOv8')
        validate_remote_model(dict(ready=True, model=detector.metadata), checkpoint)
    else:
        # Verify the predictor used for inference, not the original CPU checkpoint.
        detector.metadata['executed_device'] = str(next(detector.model.predictor.model.model.parameters()).device)
        detector.metadata['gpu_name'] = torch.cuda.get_device_name(0)
        if not detector.metadata['executed_device'].startswith('cuda'):
            raise RuntimeError('Model did not execute on GPU')
    return detector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, help='Trained checkpoint; also hash-checked in NPU mode')
    parser.add_argument('--backend', choices=('torch', 'npu'), default='torch')
    parser.add_argument('--model-manifest')
    parser.add_argument('--port', type=int, default=8775)
    parser.add_argument('--device', default='0')
    args = parser.parse_args()
    import cv2
    cv2.setNumThreads(1)
    detector = prepare_detector(args.model, args.backend, args.model_manifest, args.device)
    print(json.dumps(dict(ready=True, model=detector.metadata)), flush=True)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), service_handler(detector))
    server.daemon_threads = True
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
