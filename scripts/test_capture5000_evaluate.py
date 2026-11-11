import unittest
from types import SimpleNamespace
from capture5000_evaluate import summarize


class EvaluationMetrics(unittest.TestCase):
    def example(self):
        return SimpleNamespace(box=SimpleNamespace(ap_class_index=[3,0,2,1],
            p=[.91,.92,.93,.94],r=[.81,.82,.83,.84],ap50=[.71,.72,.73,.74],ap=[.61,.62,.63,.64]),
            results_dict={'metrics/mAP50-95(B)':.625},speed={'inference':10.})

    def test_per_class_metrics_follow_explicit_class_indices(self):
        report=summarize(self.example())
        self.assertEqual(report['classes']['mudstone_slab']['map50_95'],.61)
        self.assertEqual(report['classes']['pale_soil_patch']['recall'],.84)
        self.assertEqual(report['classes']['rock']['precision'],.92)

    def test_incomplete_unknown_or_non_finite_results_are_rejected(self):
        for change in ['missing','unknown','nan']:
            value=self.example()
            if change=='missing':value.box.ap_class_index=[3,0,2]
            elif change=='unknown':value.box.ap_class_index=[3,0,2,9]
            else:value.box.ap[0]=float('nan')
            with self.assertRaises(RuntimeError):summarize(value)


if __name__=='__main__':unittest.main()
