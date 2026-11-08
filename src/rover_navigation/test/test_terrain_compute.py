"""CPU contract tests plus real ROCm parity tests when a ROCm GPU is present."""
import importlib.util
import math
import sys
import threading
import unittest
from unittest.mock import patch

import numpy as np
from rover_navigation.core import TerrainMemory, transform, traversability
from rover_navigation.terrain_compute import CpuTerrainCompute, depth_array, RocmTerrainCompute
from rover_navigation.terrain_service import pack, unpack, dispatch, PROTOCOL, create_server


class TerrainProtocolTests(unittest.TestCase):
    def test_40_m_memory_limit_is_bounded(self):
        from rover_navigation.terrain_compute import dimensions
        self.assertEqual(dimensions(.025,40.,(0,0),12.,.03)[0],1600)
        with self.assertRaises(ValueError):dimensions(.025,40.025,(0,0),12.,.03)
    def test_binary_roundtrip_and_no_pickle(self):
        meta, data = unpack(pack(dict(operation='depth'), data=np.arange(5, dtype=np.uint8)))
        self.assertEqual(meta['operation'], 'depth')
        np.testing.assert_array_equal(data['data'], np.arange(5))
        with self.assertRaises(ValueError):
            pack({}, data=np.array([object()]))

    def test_depth_formats_padding_endianness_and_invalid_data(self):
        for big in (False, True):
            raw = np.array([[1000, 2000, 999], [0, 3000, 999]], dtype='>u2' if big else '<u2').tobytes()
            z = depth_array(raw, '16UC1', 2, 2, 6, big, [2.,0,0,0,2.,0,0,0,1.], 1)
            np.testing.assert_allclose(z, [[1.,2.],[0.,3.]])
        with self.assertRaises(ValueError):
            depth_array(b'', '32FC1', 2, 2, 8, False, [1.]*9)

    def test_cpu_reference_does_not_import_torch(self):
        with patch.dict(sys.modules, {'torch':None}):
            grid, _ = CpuTerrainCompute().classify([[.01,.01,0],[.01,.01,.1]], size=2.)
        self.assertEqual(grid.at(.01,.01), 100)

    def test_bad_protocol_operation_and_oversized_grid(self):
        with self.assertRaises(ValueError):
            dispatch(CpuTerrainCompute(), dict(protocol='old'), {})
        with self.assertRaises(ValueError):
            dispatch(CpuTerrainCompute(), dict(protocol=PROTOCOL, operation='other'), {})

    def test_binary_headers_cannot_overrun_buffers_or_create_objects(self):
        import struct,json
        for payload in (b'bad', struct.pack('<I',90000)+b'{}', pack({},v=np.arange(3))+b'extra'):
            with self.assertRaises((ValueError,KeyError)):
                unpack(payload)
        header=json.dumps(dict(meta={},arrays={'v':dict(dtype='O',shape=[1],nbytes=8)})).encode()
        with self.assertRaises(ValueError):
            unpack(struct.pack('<I',len(header))+header+b'00000000')

    def test_remote_grids_are_writable_for_clearance_seeding(self):
        from rover_navigation.terrain_service import RemoteTerrainCompute
        remote=RemoteTerrainCompute.__new__(RemoteTerrainCompute)
        meta=dict(grid=dict(resolution=.05,x=-1.,y=-1.),point_count=10)
        raw=np.zeros((4,4),np.int8).tobytes()
        def call(*args,**kw):
            return meta,dict(costs=np.frombuffer(raw,np.int8).reshape(4,4),
                             reasons=np.frombuffer(raw,np.int8).reshape(4,4))
        remote.call=call
        for grid in [remote.depth(b'')[0],remote.classify([])[0],*remote.project(TerrainMemory(2.),[0]*3,[0,0,0,1],2.,.05)]:
            grid.data[0,0]=100
            self.assertEqual(grid.data[0,0],100)

    def test_inference_failure_latches_unready(self):
        import urllib.request
        class Failing:
            metadata = dict(backend='rocm', hip='test')
            def classify(self, *args, **kw):
                raise RuntimeError('GPU failed')
        server=create_server(Failing(), 0)
        thread=threading.Thread(target=server.serve_forever, daemon=True);thread.start()
        url='http://127.0.0.1:'+str(server.server_port)
        try:
            payload=pack(dict(protocol=PROTOCOL, operation='classify'), points=np.zeros((1,3)))
            with self.assertRaises(urllib.error.HTTPError) as raised:
                urllib.request.urlopen(urllib.request.Request(url+'/compute', data=payload,
                    headers={'Content-Type':'application/octet-stream'}), timeout=2)
            self.assertEqual(raised.exception.code, 503)
            import json
            status=json.load(urllib.request.urlopen(url+'/status'))
            self.assertFalse(status['ready']); self.assertIn('GPU failed',status['error'])
        finally:
            server.shutdown();server.server_close();thread.join()


def has_rocm():
    try:
        if importlib.util.find_spec('torch') is None:
            return False
        import torch
        return bool(torch.version.hip and torch.cuda.is_available())
    except ImportError:
        return False


@unittest.skipUnless(has_rocm(), 'Requires actual AMD ROCm execution')
class RocmParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gpu=RocmTerrainCompute();cls.cpu=CpuTerrainCompute()

    def same(self, expected, actual):
        a, al=expected; b, bl=actual
        self.assertEqual((a.resolution,a.x,a.y),(b.resolution,b.x,b.y))
        np.testing.assert_array_equal(a.data < 0,b.data < 0)
        np.testing.assert_array_equal(a.data >= 90,b.data >= 90)
        # FP reduction ordering can change a rounded nonhazard cost by one.
        self.assertLessEqual(np.max(np.abs(a.data.astype(int)-b.data.astype(int))),1)
        for key in al:
            np.testing.assert_allclose(al[key], bl[key], equal_nan=True, atol=2e-7, rtol=1e-8)

    def test_empty_nan_duplicates_holes_and_unobserved_edges(self):
        rng=np.random.default_rng(31)
        clouds=[[], [[np.nan,0,0],[np.inf,0,0]], [[.01,.01,0]],
                np.repeat([[.01,.01,0],[.01,.01,.15]],1000,axis=0)]
        x,y=np.meshgrid(np.arange(-.98,.99,.0125),np.arange(-.98,.99,.0125))
        for slope in (0.,11.9999,12.0001,20.):
            p=np.column_stack([x.ravel(),y.ravel(),(x*math.tan(math.radians(slope))).ravel()])
            clouds.extend([p,p[np.hypot(p[:,0],p[:,1])>.3],p[p[:,0]>.5]])
        clouds.append(rng.uniform(-1,1,(40000,3)))
        for p in clouds:
            self.same(self.cpu.classify(p,size=2.),self.gpu.classify(p,size=2.))

    def test_depth_projection_and_nonidentity_transform(self):
        rng=np.random.default_rng(22)
        for encoding,dtype,scale in [('32FC1','f4',1),('16UC1','u2',1000)]:
            for big in (False,True):
                z=np.asarray(rng.uniform(.2,8.2,(48,65))*scale,dtype=('>' if big else '<')+dtype)
                kw=dict(encoding=encoding,width=64,height=48,step=z.strides[0],bigendian=big,
                        k=[50.,0,32.,0,50.,24.,0,0,1.],translation=[.13,-.23,.1],
                        quaternion=[0.,.1,.2,.97],size=4.,resolution=.05)
                a,al,ac=self.cpu.depth(z.tobytes(),**kw);b,bl,bc=self.gpu.depth(z.tobytes(),**kw)
                self.assertEqual(ac,bc);self.same((a,al),(b,bl))

    def test_four_sample_projection_preserves_unknown_hazard_and_reason_bits(self):
        rng=np.random.default_rng(23)
        memory=TerrainMemory(4.,.025,confirmations=1)
        memory.grid.data=rng.choice(np.array([-1,0,12,80,100],np.int8),memory.grid.data.shape)
        memory.causes[:]=rng.choice(np.array([2,4,6,8],np.int8),memory.grid.data.shape)
        for q in ([0,0,0,1],[0,0,.2,.98]):
            a,ar=self.cpu.project(memory,[.137,-.231,.1],q,4.,.05)
            b,br=self.gpu.project(memory,[.137,-.231,.1],q,4.,.05)
            np.testing.assert_array_equal(a.data,b.data);np.testing.assert_array_equal(ar.data,br.data)

    def test_invalid_geometry_and_step_thresholds(self):
        for kw in (dict(resolution=0),dict(center=(float('nan'),0)),dict(size=1e9)):
            with self.assertRaises(ValueError):self.gpu.classify([],**kw)
        for span in (.0299999,.0300001):
            p=[[.01,.01,0],[.01,.01,span]]
            self.same(self.cpu.classify(p,size=2.),self.gpu.classify(p,size=2.))


if __name__ == '__main__':
    unittest.main()
