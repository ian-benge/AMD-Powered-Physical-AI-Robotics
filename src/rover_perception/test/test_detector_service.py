from http.server import ThreadingHTTPServer
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import urlopen
import numpy as np
from rover_perception.detector_service import (RemoteDetector, prepare_detector,
                                              service_handler, validate_remote_model)
from rover_training.geometry import FOUR_CLASS_NAMES


class ServiceTests(unittest.TestCase):
    def test_real_http_preserves_bgr_and_four_class_names(self):
        images = []
        class Detector:
            metadata = dict(model='/trained.pt', backend='torch', model_sha256='a'*64,
                            class_names=list(FOUR_CLASS_NAMES))
            def detect(self, image, conf):
                images.append(image.copy())
                return SimpleNamespace(detections=[dict(class_id=3, class_name='mudstone_slab',
                    confidence=.8, xyxy=[0., 0., 2., 2.])], timings={})
        server = ThreadingHTTPServer(('127.0.0.1', 0), service_handler(Detector()))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            endpoint=f'http://127.0.0.1:{server.server_port}'
            client = RemoteDetector(endpoint, '/trained.pt')
            image = np.zeros((2, 3, 3), dtype=np.uint8)
            image[0, 0] = [11, 22, 99]
            result = client.detect(image)
            np.testing.assert_array_equal(images[0], image)
            self.assertEqual(result.detections[0]['class_name'], 'mudstone_slab')
            with self.assertRaises(ValueError):
                RemoteDetector(endpoint, '/different.pt')
        finally:
            server.shutdown()
            server.server_close()

    def npu_metadata(self, checkpoint='/trained.pt'):
        return dict(model='/candidate/model.onnx', source_checkpoint=checkpoint, backend='npu',
                    model_sha256='b'*64, source_checkpoint_sha256='a'*64,
                    class_names=list(FOUR_CLASS_NAMES), model_family='YOLOv8',
                    proof_state='npu_executed', computation_offload_verified=True,
                    compile_evidence=dict(verified=True))

    @contextmanager
    def service(self, detector):
        server = ThreadingHTTPServer(('127.0.0.1', 0), service_handler(detector))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            yield f'http://127.0.0.1:{server.server_port}'
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_npu_http_binds_checkpoint_to_artifact_and_preserves_original_boxes(self):
        detector = SimpleNamespace(metadata=self.npu_metadata(), detect=lambda image, conf: SimpleNamespace(
            detections=[dict(class_id=1, class_name='pale_soil_patch', confidence=.9, xyxy=[0., 0., 3., 2.])],
            timings=dict(preprocess_ms=.5, execution_ms=24., postprocess_ms=.8)))
        with self.service(detector) as endpoint:
            client = RemoteDetector(endpoint, '/trained.pt')
            result = client.detect(np.zeros((2, 3, 3), dtype=np.uint8))
            self.assertEqual(client.metadata['backend'], 'npu')
            self.assertEqual(result.detections[0]['xyxy'], [0., 0., 3., 2.])
            self.assertEqual(result.timings['execution_ms'], 24.)
            with urlopen(endpoint+'/status') as response:
                status = json.load(response)
            self.assertTrue(status['ready'])
            self.assertEqual(status['processed_frames'], 1)
            self.assertTrue(status['processing_session_id'])

    def test_npu_requires_actual_execution_and_matching_compilation_proof(self):
        for key, value in (('proof_state', 'assigned'), ('computation_offload_verified', False),
                           ('compile_evidence', {'verified': False}), ('model_family', 'YOLO26'),
                           ('source_checkpoint_sha256', 'bad'), ('source_checkpoint', '/other.pt')):
            with self.subTest(key=key):
                metadata = self.npu_metadata()
                metadata[key] = value
                with self.assertRaises(ValueError):
                    validate_remote_model(dict(ready=True, model=metadata), '/trained.pt')

    def test_inference_failure_latches_unready_without_backend_fallback(self):
        calls = []
        def fail(image, conf):
            calls.append(True)
            raise RuntimeError('NPU execution failed')
        with self.service(SimpleNamespace(metadata=self.npu_metadata(), detect=fail)) as endpoint:
            client = RemoteDetector(endpoint, '/trained.pt')
            for _ in range(2):
                with self.assertRaises(HTTPError) as caught:
                    client.detect(np.zeros((2, 3, 3), dtype=np.uint8))
                self.assertEqual(caught.exception.code, 503)
            with urlopen(endpoint+'/status') as response:
                status = json.load(response)
            self.assertFalse(status['ready'])
            self.assertIn('NPU execution failed', status['error'])
            self.assertEqual(status['processed_frames'], 0)
            self.assertEqual(len(calls), 1)
            with self.assertRaises(ValueError):
                RemoteDetector(endpoint, '/trained.pt')

    def test_npu_preparation_does_not_import_gpu_frameworks(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory)/'best.pt'
            checkpoint.write_bytes(b'trained-source')
            manifest = dict(source_checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                            class_names=list(FOUR_CLASS_NAMES), output_format='yolo_dfl')
            fake = SimpleNamespace(names=dict(enumerate(FOUR_CLASS_NAMES)),
                                   metadata=self.npu_metadata(str(checkpoint)), warmup=lambda: None)
            with patch('rover_perception.detectors.validate_manifest', return_value=manifest), \
                 patch('rover_perception.detectors.OnnxDetector', return_value=fake) as npu, \
                 patch('rover_perception.detectors.TorchDetector') as gpu, \
                 patch('rover_perception.npu_tools.detector_metadata', side_effect=lambda d: dict(d.metadata)), \
                 patch.dict(sys.modules, {'torch': None, 'ultralytics': None}):
                detector = prepare_detector(str(checkpoint), 'npu', '/manifest.json')
                self.assertEqual(detector.metadata['executed_device'], 'npu')
                self.assertEqual(npu.call_args.kwargs, dict(backend='npu'))
                gpu.assert_not_called()
                with self.assertRaises(ValueError):
                    prepare_detector(str(checkpoint), 'npu')
                manifest['source_checkpoint_sha256'] = 'f'*64
                with self.assertRaises(ValueError):
                    prepare_detector(str(checkpoint), 'npu', '/manifest.json')

    def test_missing_npu_provider_is_a_failure_not_a_gpu_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory)/'best.pt'
            checkpoint.write_bytes(b'trained-source')
            manifest = dict(source_checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                            class_names=list(FOUR_CLASS_NAMES), output_format='yolo_dfl')
            with patch('rover_perception.detectors.validate_manifest', return_value=manifest), \
                 patch('rover_perception.detectors.OnnxDetector', side_effect=RuntimeError('provider missing')), \
                 patch('rover_perception.detectors.TorchDetector') as gpu:
                with self.assertRaisesRegex(RuntimeError, 'provider missing'):
                    prepare_detector(str(checkpoint), 'npu', '/manifest.json')
                gpu.assert_not_called()
