"""Bounded loopback RPC for ROCm terrain compute outside the ROS container."""
import argparse
import json
import math
import struct
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from .core import Grid, TerrainMemory
from .terrain_compute import MAX_GRID_SIDE

LIMIT = 32*1024*1024
PROTOCOL = 'rover-terrain-v1'


def pack(meta, **arrays):
    descriptors, parts = {}, []
    for name, array in arrays.items():
        array = np.ascontiguousarray(array)
        if array.dtype.kind not in 'fiu' or array.dtype.itemsize not in (1,2,4,8):
            raise ValueError('Only plain numeric terrain arrays are supported')
        descriptors[name] = dict(dtype=array.dtype.str, shape=array.shape, nbytes=array.nbytes)
        parts.append(memoryview(array).cast('B'))
    header = json.dumps(dict(meta=meta, arrays=descriptors), allow_nan=False).encode()
    if len(header) > 65536:
        raise ValueError('Terrain header exceeds limit')
    payload = b''.join([struct.pack('<I',len(header)),header,*parts])
    if len(payload) > LIMIT:
        raise ValueError('Terrain payload exceeds 32 MiB')
    return payload


def unpack(payload):
    if not 4 <= len(payload) <= LIMIT:
        raise ValueError('Terrain payload exceeds 32 MiB')
    length = struct.unpack('<I',payload[:4])[0]
    if not 1 <= length <= 65536 or 4+length > len(payload):
        raise ValueError('Invalid terrain header size')
    header = json.loads(payload[4:4+length])
    if len(header['arrays']) > 16:
        raise ValueError('Too many terrain arrays')
    offset, arrays = 4+length, {}
    for name, spec in header['arrays'].items():
        dtype, shape = np.dtype(spec['dtype']), spec['shape']
        if (dtype.kind not in 'fiu' or dtype.itemsize not in (1,2,4,8) or len(shape)>3
                or any(type(v) is not int or v<0 for v in shape)):
            raise ValueError('Invalid terrain array dtype/shape')
        size = math.prod(shape)*dtype.itemsize
        if size != spec['nbytes'] or offset+size > len(payload):
            raise ValueError('Invalid terrain array length')
        arrays[name] = np.frombuffer(payload,dtype=dtype,count=math.prod(shape),offset=offset).reshape(shape)
        offset += size
    if offset != len(payload):
        raise ValueError('Trailing terrain bytes')
    return header['meta'], arrays


def grid_meta(grid):
    return dict(resolution=grid.resolution, x=float(grid.x), y=float(grid.y))


class RemoteTerrainCompute:
    def __init__(self, url='http://127.0.0.1:8782', timeout=.5):
        if url != 'http://127.0.0.1:8782':
            raise ValueError('GPU terrain service must use http://127.0.0.1:8782')
        self.url, self.timeout = url, timeout
        with urllib.request.urlopen(url+'/status', timeout=2.) as response:
            status = json.load(response)
        if (status.get('protocol') != PROTOCOL or not status.get('ready')
                or status.get('backend') != 'rocm' or not status.get('hip')):
            raise ValueError('Ready AMD ROCm terrain service required; no CPU fallback')
        self.metadata = {k:status[k] for k in ('protocol','backend','device','torch','hip','geometry','max_memory_side') if k in status}
        self.timings = {}

    def call(self, operation, meta, **arrays):
        payload = pack(dict(protocol=PROTOCOL, operation=operation, **meta), **arrays)
        started = time.monotonic()
        request = urllib.request.Request(self.url+'/compute', data=payload,
                                         headers={'Content-Type':'application/octet-stream'})
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            raw = response.read(LIMIT+1)
        meta, arrays = unpack(raw)
        if meta.get('protocol') != PROTOCOL or meta.get('backend') != 'rocm':
            raise ValueError('GPU terrain response identity mismatch')
        self.timings[operation] = dict(service_ms=meta['service_ms'], rpc_ms=(time.monotonic()-started)*1000)
        return meta, arrays

    def depth(self, data, **kw):
        meta, a = self.call('depth', kw, data=np.frombuffer(bytes(data), dtype=np.uint8))
        return Grid(a.pop('costs').copy(), **meta['grid']), a, meta['point_count']

    def classify(self, points, **kw):
        meta, a = self.call('classify', kw, points=np.asarray(points, dtype=np.float64))
        return Grid(a.pop('costs').copy(), **meta['grid']), a

    def project(self, memory, translation, quaternion, size, resolution):
        meta, a = self.call('project', dict(source=grid_meta(memory.grid), translation=list(translation),
                                            quaternion=list(quaternion), size=size, resolution=resolution),
                            costs=memory.grid.data, reasons=memory.reasons().data)
        return Grid(a['costs'].copy(), **meta['grid']), Grid(a['reasons'].copy(), **meta['grid'])


def dispatch(compute, meta, arrays):
    if meta.pop('protocol', None) != PROTOCOL:
        raise ValueError('Incompatible terrain protocol')
    operation = meta.pop('operation', None)
    if operation == 'depth':
        if not (1 <= meta['width'] <= 1920 and 1 <= meta['height'] <= 1080):
            raise ValueError('Depth image exceeds supported dimensions')
        grid, layers, count = compute.depth(arrays['data'].tobytes(), **meta)
        return dict(grid=grid_meta(grid), point_count=count), dict(costs=grid.data, **layers)
    if operation == 'classify':
        if arrays['points'].size > 6_000_000:
            raise ValueError('Point cloud exceeds supported size')
        grid, layers = compute.classify(arrays['points'], **meta)
        return dict(grid=grid_meta(grid)), dict(costs=grid.data, **layers)
    if operation == 'project':
        source = meta.pop('source')
        costs, reasons = arrays['costs'], arrays['reasons']
        if (costs.ndim != 2 or reasons.shape != costs.shape or max(costs.shape) > MAX_GRID_SIDE
                or costs.dtype != np.int8 or reasons.dtype != np.int8
                or not np.isin(costs, list(range(101))+[-1]).all()
                or not np.isfinite(list(source.values())).all() or source['resolution'] <= 0):
            raise ValueError('Invalid terrain memory')
        memory = TerrainMemory.__new__(TerrainMemory)
        memory.grid = Grid(costs, **source)
        memory.causes = reasons
        grid, flags = compute.project(memory, **meta)
        return dict(grid=grid_meta(grid)), dict(costs=grid.data, reasons=flags.data)
    raise ValueError('Unknown terrain operation')


def create_server(compute, port=8782):
    lock = threading.Lock()
    state = dict(ready=True, error=None, completed=0, operations={}, protocol=PROTOCOL,
                 max_memory_side=MAX_GRID_SIDE, **compute.metadata)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, data, content='application/json'):
            raw = json.dumps(data).encode() if content == 'application/json' else data
            self.send_response(code)
            self.send_header('Content-Type', content)
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            self.reply(200 if self.path == '/status' else 404, state)

        def do_POST(self):
            acquired = False
            try:
                if self.path != '/compute':
                    return self.reply(404, dict(error='Not found'))
                length = int(self.headers.get('Content-Length', '0'))
                if not 1 <= length <= LIMIT or self.headers.get('Content-Type') != 'application/octet-stream':
                    raise ValueError('Invalid terrain payload')
                acquired = lock.acquire(blocking=False)
                if not acquired:
                    return self.reply(429, dict(error='One terrain request at a time; discard old work'))
                if not state['ready']:
                    return self.reply(503, dict(error=state['error']))
                self.connection.settimeout(1.)
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise ValueError('Truncated terrain request')
                meta, arrays = unpack(raw)
                operation = meta.get('operation')
                started = time.monotonic()
                try:
                    result, output = dispatch(compute, meta, arrays)
                except (ValueError, KeyError, TypeError, IndexError):
                    raise
                except Exception as error:
                    state.update(ready=False, error=f'{type(error).__name__}: {error}; restart GPU service')
                    return self.reply(503, dict(error=state['error']))
                elapsed = (time.monotonic()-started)*1000
                payload = pack(dict(protocol=PROTOCOL, backend='rocm', service_ms=elapsed, **result), **output)
                state['completed'] += 1
                state['operations'][operation] = dict(service_ms=elapsed)
                self.reply(200, payload, 'application/octet-stream')
            except (ValueError, KeyError, TypeError, IndexError) as error:
                self.reply(400, dict(error=str(error)))
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass
            finally:
                if acquired:
                    lock.release()
    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8782)
    args = parser.parse_args()
    from .terrain_compute import RocmTerrainCompute
    compute = RocmTerrainCompute()
    # Complete real GPU kernels and read-back before exposing readiness.
    compute.classify([[.01,.01,0.], [.01,.01,.15]], size=2.)
    server = create_server(compute, args.port)
    print(json.dumps(dict(ready=True, port=args.port, **compute.metadata)), flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
