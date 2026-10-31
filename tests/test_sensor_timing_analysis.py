"""Synthetic observations independent of the library's delay scheduler."""
import importlib.util
from pathlib import Path
import unittest
import sys
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('analysis',Path(__file__).resolve().parents[1]/'tools/analyze_sensor_timing.py')
m=importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
class Analysis(unittest.TestCase):
    def records(self,kind='exposure',delay=3):
        writes=[{'type':'write','sequence':i,'target':i+2,'error':0} for i in (14,30,46,62,78)]
        frames=[]
        timestamp=0
        for seq in range(96):
            level=0.2 if sum(w['sequence']+delay<=seq for w in writes)%2==0 else 0.6
            frames.append({'type':'frame','sequence':seq,'timestamp_ns':timestamp,'luminance':level})
            timestamp+=60000000 if level<0.4 else 90000000
        return [{'type':'configuration','diagnostic':kind,'status':0,'trace_dropped':0}]+frames+writes
    def test_exposure(self):
        result=m.analyze(self.records());self.assertTrue(result['valid']);self.assertEqual(result['suggested_delay'],3)
    def test_frame_interval(self):
        result=m.analyze(self.records('vblank',2));self.assertTrue(result['valid']);self.assertEqual(result['suggested_delay'],2)
    def test_flat(self):
        r=self.records()
        for f in r:
            if f['type']=='frame':f['luminance']=0.3
        self.assertFalse(m.analyze(r)['valid'])
    def test_missing_frames(self):
        r=self.records();del r[20];self.assertFalse(m.analyze(r)['valid'])
    def test_failed_capture(self):
        r=self.records();r[0]['status']=-62;self.assertFalse(m.analyze(r)['valid'])
    def test_lost_trace(self):
        r=self.records();r[0]['trace_dropped']=1;self.assertFalse(m.analyze(r)['valid'])
if __name__=='__main__':unittest.main()
