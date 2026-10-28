"""Optional process resource observations; CPU/RSS are software, not machine data."""
import threading
import time


class ResourceProbe:
    def __init__(self):
        self.event=threading.Event(); self.peak_rss=0; self.process=None; self.thread=None

    def start(self):
        self.started=time.perf_counter(); self.cpu_started=time.process_time()
        try:
            import psutil
            self.process=psutil.Process()
        except ImportError: return self
        def sample():
            while not self.event.is_set():
                try: self.peak_rss=max(self.peak_rss,self.process.memory_info().rss)
                except Exception: break
                self.event.wait(.05)
        self.thread=threading.Thread(target=sample,daemon=True); self.thread.start(); return self

    def stop(self):
        self.event.set()
        if self.thread: self.thread.join(timeout=1.)
        return {'scope':'CURRENT_PYTHON_PROCESS','wall_seconds':time.perf_counter()-self.started,
                'cpu_seconds':time.process_time()-self.cpu_started,'peak_rss_bytes':self.peak_rss or None,
                'rss_sampling_seconds':.05 if self.process else None,
                'status':'OBSERVED' if self.process else 'PARTIAL_NO_PSUTIL'}
