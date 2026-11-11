import datetime as dt
import json
import unittest
import autonomous_demo_reboot_recovery as recovery


class RebootReviewTests(unittest.TestCase):
    def row(self,message,boot=None,transport='kernel',seconds=100):
        return json.dumps(dict(_BOOT_ID=(boot or recovery.OLD_BOOT).replace('-',''),_TRANSPORT=transport,
            MESSAGE=message,__REALTIME_TIMESTAMP=str(seconds*1_000_000)))
    def test_protected_journal_requires_matching_boot_transport_and_nonempty_evidence(self):
        for text in ('',self.row('normal',recovery.BOOT_CHAIN[-1]),self.row('normal',transport='stdout')):
            with self.assertRaises(RuntimeError):recovery.journal_review(text,recovery.OLD_BOOT)
        self.assertEqual(recovery.journal_review(self.row('normal initialization'),recovery.OLD_BOOT),[])
    def test_gpu_and_kernel_incidents_are_reported(self):
        for message in ('amdgpu_job_timedout','GPU reset begin','kernel panic','soft lockup','oom-kill'):
            with self.subTest(message=message):self.assertEqual(len(recovery.journal_review(self.row(message),recovery.OLD_BOOT)),1)
    def test_old_boot_incidents_are_scoped_to_the_actual_unfinished_session(self):
        since=dt.datetime.fromtimestamp(150,dt.timezone.utc).isoformat()
        text=self.row('GPU reset begin',seconds=100)+'\n'+self.row('normal',seconds=160)
        self.assertEqual(recovery.journal_review(text,recovery.OLD_BOOT,since),[])
        self.assertEqual(len(recovery.journal_review(text+'\n'+self.row('GPU reset begin',seconds=170),recovery.OLD_BOOT,since)),1)
    def test_cooldown_requires_real_span_and_all_temperatures_below_limit(self):
        epoch=dt.datetime.now(dt.timezone.utc)
        samples=[dict(utc=(epoch+dt.timedelta(seconds=i*5)).isoformat(),tctl_c=30.) for i in range(13)]
        self.assertTrue(recovery.cooldown_pass(samples))
        self.assertFalse(recovery.cooldown_pass(samples[:6]))
        samples[-1]['tctl_c']=71.;self.assertFalse(recovery.cooldown_pass(samples))
    def report(self):
        return dict(status='REVIEWED_REBOOT_WITH_MISSING_TEARDOWN',fresh_capture_preparation_permitted=True,
            case=str(recovery.CASE),recovery_boot_id=recovery.BOOT_CHAIN[-1],boot_chain=recovery.BOOT_CHAIN,
            selection_sha256=recovery.SELECTION_SHA,post_teardown_clean=False,monitoring_gap=True,
            original_teardown_completed=False,finished_utc=dt.datetime.now(dt.timezone.utc).isoformat())
    def test_review_cannot_claim_clean_teardown_or_reuse_wrong_identity(self):
        recovery.verify_report(self.report())
        for field,value in [('post_teardown_clean',True),('monitoring_gap',False),('recovery_boot_id','other'),
            ('case','other'),('selection_sha256','other'),('status','BLOCKED')]:
            report=self.report();report[field]=value
            with self.subTest(field=field),self.assertRaises(RuntimeError):recovery.verify_report(report)
    def test_old_review_expires_before_capture_preparation(self):
        report=self.report();report['finished_utc']=(dt.datetime.now(dt.timezone.utc)-dt.timedelta(minutes=11)).isoformat()
        with self.assertRaises(RuntimeError):recovery.verify_report(report)


if __name__=='__main__':unittest.main()
