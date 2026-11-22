import ast
import unittest

from prepare_halo_gpu_candidate import terrain_candidate


class TerrainCandidateTests(unittest.TestCase):
    def fixture(self):
        return """import os
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'bulk10000-collection-20261106'))
def worker():
    if any(k in ('LD_PRELOAD','LD_LIBRARY_PATH','PYTHONPATH') or k.startswith(('HSA_','HIP_','ROCR_')) for k in os.environ):
        raise RuntimeError('Unexpected terrain runtime override')
    save(condition='environment-v2-1600-grid-bound-stock-7141-blocking-sync',)
    start_service(RocmTerrainCompute,create_server,hip,8782,save)
def main():
    env.update(OMP_NUM_THREADS='1',ROCM_SDK_TARGET_FAMILY='gfx1151')
    print('ENVIRONMENT_V2_TERRAIN_READY: float64, HIP requested/observed 4, grid bound 1600')
"""

    def test_candidate_rejects_wrong_policy_and_unrelated_overrides_before_gpu(self):
        generated = terrain_candidate(self.fixture())
        tree = ast.parse(generated)
        worker = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'worker')
        # Execute only the two environment gates: never import Torch or load HIP.
        gates = ast.Module(body=worker.body[:2], type_ignores=[])
        code = compile(gates, '<environment gates>', 'exec')
        from types import SimpleNamespace
        exec(code, {'os': SimpleNamespace(environ={'HSA_ENABLE_SDMA': '0'})})
        for environment in ({}, {'HSA_ENABLE_SDMA': '1'}, {'HSA_ENABLE_SDMA': '0', 'HIP_VISIBLE_DEVICES': '0'},
                            {'HSA_ENABLE_SDMA': '0', 'LD_PRELOAD': 'bad'}, {'HSA_ENABLE_SDMA': '0', 'HSA_XNACK': '1'}):
            with self.subTest(environment=environment), self.assertRaises(RuntimeError):
                exec(code, {'os': SimpleNamespace(environ=environment)})

    def test_unknown_retained_source_shape_refused(self):
        with self.assertRaises(ValueError):
            terrain_candidate(self.fixture().replace("OMP_NUM_THREADS='1'", "OMP_NUM_THREADS='2'"))


if __name__ == '__main__':
    unittest.main()
