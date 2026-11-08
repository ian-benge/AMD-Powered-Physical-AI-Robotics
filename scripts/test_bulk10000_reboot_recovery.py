"""CPU-only recovery policy checks; no workload, root command or device access."""
import datetime as dt
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import unittest
from unittest.mock import patch

import bulk10000_reboot_recovery as r


class RecoveryPolicy(unittest.TestCase):
    def journal(self,message='normal',boot=r.OLD_BOOT):
        return json.dumps(dict(_BOOT_ID=boot.replace('-',''),_TRANSPORT='kernel',MESSAGE=message))

    def test_old_kernel_empty_or_wrong_boot_is_blocked(self):
        for text in ['',self.journal(boot='a'*32)]:
            with self.subTest(text=text), self.assertRaises(RuntimeError):r.journal_review(text,r.OLD_BOOT)

    def test_preserves_userspace_crash_without_calling_it_a_gpu_reset(self):
        self.assertEqual(r.journal_review(self.journal('ruby: segfault in RenderSystem_GL.so'),r.OLD_BOOT),[])

    def test_gpu_kernel_incident_is_not_recoverable_automatically(self):
        for message in ['amdgpu_job_timedout','GPU reset begin','GPU fault','out of memory']:
            self.assertEqual(len(r.journal_review(self.journal(message),r.OLD_BOOT)),1)

    def test_non_kernel_records_rejected(self):
        with self.assertRaises(RuntimeError):
            r.journal_review(self.journal().replace('kernel','stdout'),r.OLD_BOOT)

    def test_cooldown_requires_seven_distinct_spaced_cool_readings(self):
        start=dt.datetime(2026,11,7,tzinfo=dt.timezone.utc)
        rows=[dict(utc=(start+dt.timedelta(seconds=5*i)).isoformat(),tctl_c=40) for i in range(7)]
        self.assertTrue(r.cooled(rows))
        self.assertFalse(r.cooled(rows[:6]))
        self.assertFalse(r.cooled(rows[:-1]+[dict(rows[-1],tctl_c=71)]))
        self.assertFalse(r.cooled([rows[0]]*7))
        self.assertFalse(r.cooled(rows[:-1]+[dict(rows[-1],tctl_c=-30)]))

    def test_no_review_without_explicit_local_root_acknowledgement(self):
        with patch.object(r.os,'geteuid',return_value=1001,create=True):
            with self.assertRaises(RuntimeError):r.review(Path('/unused'),True)
        with patch.object(r.os,'geteuid',return_value=0,create=True),patch.object(r.os,'isatty',return_value=True):
            with self.assertRaises(RuntimeError):r.review(Path('/unused'),False)

    def test_idle_workload_or_inspection_failure_is_blocked(self):
        for value in [dict(returncode=0,stdout='container\n'),dict(returncode=1,stdout='')]:
            with patch.object(r,'command',return_value=value),self.assertRaises(RuntimeError):r.idle()

    def test_evidence_save_never_overwrites(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'record.json';r.save(path,{'original':True})
            with self.assertRaises(FileExistsError):r.save(path,{'original':False})
            self.assertEqual(r.read(path),{'original':True})

    def test_recovery_does_not_run_for_same_boot(self):
        with self.assertRaises(RuntimeError):r.unchanged_selection(r.OLD_BOOT)

    def test_current_selection_cannot_be_replaced_if_changed(self):
        with patch.object(r,'sha',return_value='changed'),self.assertRaises(RuntimeError):
            r.unchanged_selection('new-boot')

    def test_review_must_record_gap_and_match_case_and_both_boots(self):
        report=dict(status='REVIEWED_REBOOT_WITH_MISSING_TEARDOWN',
            fresh_session_preparation_permitted=True,old_boot_id=r.OLD_BOOT,
            recovery_boot_id='new-boot',case=str(r.CASE),original_selection_sha256=r.SELECTION_SHA,
            post_teardown_clean=False,monitoring_gap=True)
        r.review_identity(report,'new-boot')
        for key,value in [('status','BLOCKED'),('fresh_session_preparation_permitted',False),
                ('old_boot_id','wrong'),('recovery_boot_id','wrong'),('case','/other-case'),
                ('original_selection_sha256','wrong'),('post_teardown_clean',True),('monitoring_gap',False)]:
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                r.review_identity(dict(report,**{key:value}),'new-boot')

    def test_unprivileged_review_file_cannot_authorize_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);r.save(folder/'PROTECTED_REVIEW.json',{})
            with self.assertRaises(RuntimeError):r.verify_review(folder,'new-boot')

    @unittest.skipUnless(sys.platform.startswith('linux'),'Linux group permissions')
    def test_published_evidence_keeps_owner_and_hashes_with_group_read_access(self):
        with tempfile.TemporaryDirectory() as directory:
            evidence=Path(directory)/'protected-review-test';evidence.mkdir(mode=0o700)
            files=[evidence/'saved.json',evidence/'kernel.jsonl']
            for path in files:
                path.write_text('unchanged evidence');path.chmod(0o600)
            owners={p:p.stat().st_uid for p in files+[evidence]}
            hashes={p:r.sha(p) for p in files}
            r.publish_review_evidence(evidence,files,os.getgid())
            self.assertEqual(stat.S_IMODE(evidence.stat().st_mode),0o750)
            for path in files:
                self.assertEqual(stat.S_IMODE(path.stat().st_mode),0o640)
                self.assertEqual(r.sha(path),hashes[path])
            for path in files+[evidence]:
                self.assertEqual(path.stat().st_uid,owners[path])
                self.assertEqual(path.stat().st_gid,os.getgid())

    @unittest.skipUnless(sys.platform.startswith('linux'),'Linux evidence paths')
    def test_permission_publication_rejects_symlinks_and_outside_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);evidence=root/'review';evidence.mkdir(mode=0o700)
            outside=root/'outside';outside.write_text('preserve')
            link=evidence/'link';link.symlink_to(outside)
            mode=outside.stat().st_mode
            for paths in [[outside],[link]]:
                with self.assertRaises(RuntimeError):r.publish_review_evidence(evidence,paths,os.getgid())
            self.assertEqual(outside.stat().st_mode,mode)
            self.assertEqual(stat.S_IMODE(evidence.stat().st_mode),0o700)


if __name__ == '__main__':unittest.main()
