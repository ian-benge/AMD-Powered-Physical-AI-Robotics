import datetime as dt
import json
import unittest

from halo_gpu_incident_recovery import compare_journal, cooled


class IncidentReviewTests(unittest.TestCase):
    def rows(self, messages, boot='abc'):
        return '\n'.join(json.dumps(dict(_BOOT_ID=boot, _TRANSPORT='kernel',
                     __REALTIME_TIMESTAMP=str(i), MESSAGE=m)) for i, m in enumerate(messages))

    def test_known_incident_retained_and_ordinary_later_entries_allowed(self):
        old = self.rows(['ring sdma0 timeout', 'GPU reset succeeded'])
        new = self.rows(['ring sdma0 timeout', 'GPU reset succeeded', 'ordinary message'])
        self.assertEqual(len(compare_journal(new, old, 'abc')), 2)

    def test_new_reset_or_fault_refuses_recovery(self):
        old = self.rows(['ring sdma0 timeout', 'GPU reset succeeded'])
        for message in ['ring sdma0 timeout', 'GPU page fault', 'MES failed to respond', 'oom-kill']:
            with self.subTest(message=message), self.assertRaises(RuntimeError):
                compare_journal(old + '\n' + self.rows([message]), old, 'abc')

    def test_missing_incident_wrong_boot_and_empty_journal_refused(self):
        old = self.rows(['ring sdma0 timeout', 'GPU reset succeeded'])
        for new in ['', self.rows(['GPU reset succeeded']), old.replace('abc', 'def')]:
            with self.assertRaises(RuntimeError):
                compare_journal(new, old, 'abc')

    def test_cooldown_requires_continuous_fresh_below_threshold_samples(self):
        start = dt.datetime(2026, 11, 17, tzinfo=dt.timezone.utc)
        samples = [dict(utc=(start + dt.timedelta(seconds=i * 5)).isoformat(), tctl_c=30.) for i in range(7)]
        self.assertTrue(cooled(samples))
        self.assertFalse(cooled(samples[:6]))
        self.assertFalse(cooled([*samples[:-1], dict(samples[-1], tctl_c=71.)]))
        self.assertFalse(cooled([*samples[:-1], dict(samples[-1], utc=(start + dt.timedelta(seconds=60)).isoformat())]))


if __name__ == '__main__':
    unittest.main()
