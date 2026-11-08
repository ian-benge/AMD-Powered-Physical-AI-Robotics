"""An actual terminal-group interrupt must not interrupt the health subprocess."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[3]


@unittest.skipIf(os.name=='nt','POSIX terminal process-group check runs on Halo')
class DesktopStopTests(unittest.TestCase):
    def test_ctrl_c_stops_supervisor_without_interrupting_its_current_health_check(self):
        code='''import signal,subprocess,sys
stopping=False
def stop(*_):
 global stopping
 stopping=True
signal.signal(signal.SIGINT,stop)
print('CHECK_START',flush=True)
result=subprocess.run([sys.executable,'-c','import time; time.sleep(.5)'])
assert result.returncode==0,'Health query received the terminal interrupt'
assert stopping,'Supervisor did not receive stop'
print('CLEAN_STOP',flush=True)
'''
        with tempfile.TemporaryDirectory() as folder:
            script=Path(folder)/'fake_supervisor.py';script.write_text(code)
            child=subprocess.Popen([sys.executable,str(ROOT/'scripts/phase2_desktop_launch.py'),str(script)],
                                   start_new_session=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            try:
                self.assertEqual(child.stdout.readline().strip(),'CHECK_START')
                time.sleep(.1)
                os.killpg(child.pid,signal.SIGINT)
                output,error=child.communicate(timeout=10)
                self.assertEqual(child.returncode,0,error)
                self.assertIn('CLEAN_STOP',output)
                self.assertEqual(error,'')
            finally:
                if child.poll() is None:child.kill();child.wait()
