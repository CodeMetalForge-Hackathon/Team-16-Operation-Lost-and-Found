"""Runs the Hailo inference in its OWN process so a hung chip can never freeze the cameras or the dashboard.

fusion_pi.py talks to HailoProxy (same detect(img, color) call as before). The real FusionHailo lives in a
worker process. If the worker does not answer within TIMEOUT the proxy returns "no detections" immediately
(the video keeps flowing, the status line says AI STALLED), and after RESTART_AFTER seconds the worker is
killed and a fresh one is started. A worker that dies or cannot open the chip is retried every RETRY_DEAD s.
"""
import multiprocessing as mp
import time

TIMEOUT = 2.0          # a normal inference takes well under 0.1 s
RESTART_AFTER = 6.0    # stalled this long: kill the worker and start another
START_TIMEOUT = 60.0   # worker must report ready within this
RETRY_DEAD = 30.0      # worker could not open the chip: try again this often
EMPTY = (0.0, {}, [])


def worker_main(conn, hef):
    """Entry point of the worker process (spawned, so it starts clean with no camera threads)."""
    try:
        from fusion_pi import FusionHailo
        h = FusionHailo(hef)
    except Exception as e:  # noqa: BLE001 - report anything, the parent decides what to do
        try:
            conn.send(("fatal", f"{type(e).__name__}: {e}"))
        except (OSError, BrokenPipeError):
            pass
        return
    conn.send(("ready",))
    while True:
        try:
            msg = conn.recv()
        except (EOFError, OSError):
            break  # parent is gone: release the chip cleanly
        if msg is None:
            break
        img, color = msg
        try:
            res = h.detect(img, color)
        except Exception as e:  # noqa: BLE001
            print(f"hailo worker: detect failed: {e}", flush=True)
            res = EMPTY
        try:
            conn.send(("res", res))
        except (OSError, BrokenPipeError):
            break
    try:
        h.close()
    except Exception:  # noqa: BLE001
        pass


class HailoProxy:
    name = "hailo-proc"

    def __init__(self, hef):
        self.hef = hef
        self.ctx = mp.get_context("spawn")
        self.proc = self.conn = None
        self.state = "starting"      # starting | ready | stalled | dead
        self.t_spawn = self.t_busy = self.t_dead = 0.0
        self.restarts = 0
        self._spawn()

    # ---- process management
    def _spawn(self):
        parent, child = self.ctx.Pipe()
        self.proc = self.ctx.Process(target=worker_main, args=(child, self.hef), daemon=True)
        self.proc.start()
        child.close()
        self.conn, self.state, self.t_spawn = parent, "starting", time.time()
        print("hailo worker started pid", self.proc.pid, flush=True)

    def _kill(self):
        try:
            if self.proc is not None and self.proc.is_alive():
                self.proc.terminate()          # polite first
                self.proc.join(1.5)
                if self.proc.is_alive():
                    self.proc.kill()           # a hung one ignores SIGTERM
                    self.proc.join(2.0)
        except Exception:  # noqa: BLE001
            pass
        try:
            if self.conn is not None:
                self.conn.close()
        except Exception:  # noqa: BLE001
            pass
        self.proc = self.conn = None

    def _respawn(self, why):
        self.restarts += 1
        print(f"hailo worker: {why} - restarting it (restart #{self.restarts})", flush=True)
        self._kill()
        self._spawn()

    # ---- the call used by fusion_pi.py
    def detect(self, img, color):
        now = time.time()
        if self.state == "dead":
            if now - self.t_dead >= RETRY_DEAD:
                self._spawn()
            return EMPTY
        if self.proc is not None and not self.proc.is_alive():
            print("hailo worker: process died", flush=True)
            self._kill()
            self.state, self.t_dead = "dead", now
            return EMPTY
        if self.state == "starting":
            while self.conn.poll(0):
                kind = self.conn.recv()
                if kind[0] == "ready":
                    self.state = "ready"
                    print("hailo worker ready", flush=True)
                elif kind[0] == "fatal":
                    print("hailo worker could not open the chip:", kind[1], flush=True)
                    self._kill()
                    self.state, self.t_dead = "dead", now
                    return EMPTY
            if self.state == "starting":
                if now - self.t_spawn > START_TIMEOUT:
                    self._respawn("did not become ready")
                return EMPTY
        if self.state == "stalled":
            if self.conn.poll(0):              # it answered late: it is alive again
                self.conn.recv()
                self.state = "ready"
                print("hailo worker recovered by itself", flush=True)
            elif now - self.t_busy > RESTART_AFTER:
                self._respawn(f"no answer for {now - self.t_busy:.0f}s")
                return EMPTY
            else:
                return EMPTY
        # state == ready: one request, wait up to TIMEOUT
        self.conn.send((img, color))
        self.t_busy = now
        if self.conn.poll(TIMEOUT):
            kind = self.conn.recv()
            if kind[0] == "res":
                return kind[1]
            return EMPTY
        self.state = "stalled"
        print(f"hailo worker: no answer in {TIMEOUT:.0f}s - video continues without AI", flush=True)
        return EMPTY

    def close(self):
        try:
            if self.conn is not None and self.state in ("ready", "starting"):
                self.conn.send(None)           # let it release the chip itself
                self.proc.join(4.0)
        except Exception:  # noqa: BLE001
            pass
        self._kill()
