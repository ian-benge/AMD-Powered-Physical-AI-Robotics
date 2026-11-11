import copy
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from demo_exterior_view import run_gui,stationary,token_path,worker_command


class ExteriorViewTests(unittest.TestCase):
    def test_exec_worker_sources_ros_and_keeps_arguments_separate(self):
        command=worker_command('owned-container','a'*32,'--worker','--seconds','60')
        self.assertEqual(command[:3],['docker','exec','-e'])
        self.assertIn('source /opt/ros/jazzy/setup.bash',command[7])
        self.assertIn('source /work/install/setup.bash',command[7])
        self.assertEqual(command[-3:],['--worker','--seconds','60'])
    def test_only_fresh_stopped_disarmed_views_are_allowed(self):
        good=dict(status=dict(mode='stopped',fault=None,command=[0.,0.],age_seconds=.1),
                  camera_health=dict(available=True,age_seconds=.1),demo_status=dict(goal_active=False),
                  localization=dict(healthy=True))
        self.assertTrue(stationary(good))
        for section,key,value in [('status','mode','manual'),('status','command',[.01,0]),
                ('status','age_seconds',1.),('status','command',[float('nan'),0]),('localization','healthy',False),
                ('demo_status','goal_active',True),('camera_health','available',False),('camera_health','age_seconds',1.)]:
            changed=copy.deepcopy(good);changed[section][key]=value
            with self.subTest(section=section,key=key):self.assertFalse(stationary(changed))
        with self.assertRaises(ValueError):token_path('../other')
        faulted=copy.deepcopy(good);faulted['status']['fault']=dict(reason='invalid_odometry',sim_time=.026)
        preserved=copy.deepcopy(faulted)
        self.assertTrue(stationary(faulted));self.assertEqual(faulted,preserved)

    def test_cancel_before_start_launches_nothing(self):
        event=threading.Event();event.set()
        self.assertEqual(run_gui(['/no-such-program'],10,event),'cancelled_before_launch')

    @unittest.skipUnless(sys.platform=='linux','Linux PID ownership and cleanup')
    def test_timeout_reaps_a_detached_child_without_touching_same_partition_peer(self):
        with tempfile.TemporaryDirectory() as directory:
            pidfile=Path(directory)/'child.pid'
            code=('import subprocess,sys,time; from pathlib import Path; '
                  'p=subprocess.Popen([sys.executable,"-c","import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(60)"],start_new_session=True); '
                  'Path(sys.argv[1]).write_text(str(p.pid));time.sleep(60)')
            peer=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],
                                  env=dict(os.environ,GZ_PARTITION='shared-current-world'))
            try:
                with patch.dict(os.environ,GZ_PARTITION='shared-current-world'):
                    outcome=run_gui([sys.executable,'-c',code,str(pidfile)],.3,threading.Event())
                self.assertEqual(outcome,'bounded_timeout');self.assertIsNone(peer.poll())
                child=Path('/proc')/pidfile.read_text().strip()
                if child.exists():self.assertEqual((child/'stat').read_text().rsplit(')',1)[1].split()[0],'Z')
            finally:peer.terminate();peer.wait(timeout=5)

    @unittest.skipUnless(sys.platform=='linux','Linux process ownership')
    def test_operator_stop_reaps_viewer(self):
        event=threading.Event();timer=threading.Timer(.2,event.set);timer.start()
        try:self.assertEqual(run_gui([sys.executable,'-c','import time;time.sleep(60)'],20,event),'operator_stop')
        finally:timer.cancel();timer.join()


if __name__=='__main__':unittest.main()
