"""Numpad keyboard controller using a low-level Windows keyboard hook.

Uses the same on_press / on_release callback style as pynput, but installs a
WH_KEYBOARD_LL hook that *suppresses* numpad key events before they reach the
MuJoCo/GLFW window.  Without suppression, held numpad keys flood the GLFW
message queue with WM_KEYDOWN auto-repeat events; GLFW holds its internal mutex
while draining them, which causes viewer.sync() on the main thread to block.

All non-numpad keys (ESC, F-keys, etc.) are forwarded normally so MuJoCo viewer
shortcuts keep working.

Requires NumLock ON.

Key layout:
  7(yaw←)   8(forward)  9(yaw→)
  4(left)   5(→LOCO)    6(right)
  1(BeyondMimic) 2(back) 3(TrackMimic)
             0(POS_RESET)
"""
import ctypes
import ctypes.wintypes
import threading

# ── Windows constants ──────────────────────────────────────────────────────────
_WH_KEYBOARD_LL = 13
_WM_KEYDOWN     = 0x0100
_WM_KEYUP       = 0x0101
_WM_SYSKEYDOWN  = 0x0104
_WM_SYSKEYUP    = 0x0105
_WM_QUIT        = 0x0012
_HC_ACTION      = 0

# VK codes for numpad 0-9 (require NumLock ON)
_NUMPAD_VKS = set(range(0x60, 0x6A))   # 0x60=NUM0 … 0x69=NUM9


class NumKey:
    NUM0 = 0x60
    NUM1 = 0x61
    NUM2 = 0x62
    NUM3 = 0x63
    NUM4 = 0x64
    NUM5 = 0x65
    NUM6 = 0x66
    NUM7 = 0x67
    NUM8 = 0x68
    NUM9 = 0x69


# ── 64-bit-safe types (wintypes.LPARAM is c_long=32-bit, wrong on 64-bit Windows)
_LPARAM  = ctypes.c_longlong    # LONG_PTR  on 64-bit Windows
_WPARAM  = ctypes.c_ulonglong   # ULONG_PTR on 64-bit Windows
_LRESULT = ctypes.c_longlong

# ── ctypes structs ─────────────────────────────────────────────────────────────
class _KBDLLHOOKSTRUCT(ctypes.Structure):
    _fields_ = [
        ('vkCode',      ctypes.c_uint32),
        ('scanCode',    ctypes.c_uint32),
        ('flags',       ctypes.c_uint32),
        ('time',        ctypes.c_uint32),
        ('dwExtraInfo', ctypes.POINTER(ctypes.c_ulong)),
    ]

_HOOKPROC = ctypes.WINFUNCTYPE(_LRESULT, ctypes.c_int, _WPARAM, _LPARAM)

# Pre-declare argtypes so ctypes uses the correct 64-bit marshalling
_CallNextHookEx = ctypes.windll.user32.CallNextHookEx
_CallNextHookEx.argtypes = [ctypes.c_void_p, ctypes.c_int, _WPARAM, _LPARAM]
_CallNextHookEx.restype  = _LRESULT

_SetWindowsHookExW = ctypes.windll.user32.SetWindowsHookExW
_SetWindowsHookExW.argtypes = [ctypes.c_int, _HOOKPROC, ctypes.wintypes.HINSTANCE, ctypes.wintypes.DWORD]
_SetWindowsHookExW.restype  = ctypes.c_void_p

# ── Shared state (mirrors pynput key_states pattern) ──────────────────────────
_key_states:       dict[int, bool] = {vk: False for vk in _NUMPAD_VKS}
_pending_pressed:  set[int]        = set()
_pending_released: set[int]        = set()
_lock = threading.Lock()

# ── on_press / on_release callbacks (pynput-style) ────────────────────────────
def _on_press(vk: int) -> None:
    with _lock:
        if not _key_states[vk]:         # ignore auto-repeat
            _pending_pressed.add(vk)
            _key_states[vk] = True


def _on_release(vk: int) -> None:
    with _lock:
        _key_states[vk] = False
        _pending_released.add(vk)


# ── Hook implementation ────────────────────────────────────────────────────────
_hook        = None
_proc_ref    = None     # prevent HOOKPROC from being GC'd
_thread_id   = 0
_hook_ready  = threading.Event()


def _hook_proc(nCode: int, wParam: int, lParam: int) -> int:
    try:
        if nCode == _HC_ACTION:
            kbd = ctypes.cast(lParam, ctypes.POINTER(_KBDLLHOOKSTRUCT)).contents
            vk  = kbd.vkCode
            if vk in _NUMPAD_VKS:
                if wParam in (_WM_KEYDOWN, _WM_SYSKEYDOWN):
                    _on_press(vk)
                elif wParam in (_WM_KEYUP, _WM_SYSKEYUP):
                    _on_release(vk)
                # Return 1 → suppress event; GLFW never sees it
                return 1
    except Exception:
        pass
    # Non-numpad key → forward to next hook / GLFW as usual
    return _CallNextHookEx(_hook, nCode, wParam, lParam)


def _hook_thread() -> None:
    global _hook, _proc_ref, _thread_id
    _thread_id = ctypes.windll.kernel32.GetCurrentThreadId()

    proc      = _HOOKPROC(_hook_proc)
    _proc_ref = proc                    # keep alive
    _hook     = _SetWindowsHookExW(_WH_KEYBOARD_LL, proc, 0, 0)
    _hook_ready.set()

    if not _hook:
        return

    msg = ctypes.wintypes.MSG()
    while ctypes.windll.user32.GetMessageW(ctypes.byref(msg), None, 0, 0) != 0:
        ctypes.windll.user32.TranslateMessage(ctypes.byref(msg))
        ctypes.windll.user32.DispatchMessageW(ctypes.byref(msg))

    ctypes.windll.user32.UnhookWindowsHookEx(_hook)
    _hook = None


# Start hook thread at import time (daemon → auto-killed when main exits)
_thread = threading.Thread(target=_hook_thread, daemon=True, name='NumpadHookThread')
_thread.start()
_hook_ready.wait(timeout=3.0)


# ── Public class ───────────────────────────────────────────────────────────────
class NumpadKeyboard:
    """Read numpad key states captured by the module-level WH_KEYBOARD_LL hook.

    Call update() once per control-loop iteration to snapshot edge events.
    """

    def __init__(self, speed: float = 1.0):
        self.speed = speed
        self._just_pressed:  set[int] = set()
        self._just_released: set[int] = set()

    def update(self):
        """Snapshot pending press/release events. Call once per loop iteration."""
        global _pending_pressed, _pending_released
        with _lock:
            self._just_pressed  = _pending_pressed.copy()
            self._just_released = _pending_released.copy()
            _pending_pressed.clear()
            _pending_released.clear()

    def is_pressed(self, vk: int) -> bool:
        with _lock:
            return _key_states.get(vk, False)

    def is_just_pressed(self, vk: int) -> bool:
        return vk in self._just_pressed

    def is_released(self, vk: int) -> bool:
        return vk in self._just_released

    def get_vel_cmd(self):
        """Return (forward, lateral, yaw) from currently held movement keys."""
        with _lock:
            fwd = lat = yaw = 0.0
            if _key_states[NumKey.NUM8]: fwd += self.speed
            if _key_states[NumKey.NUM2]: fwd -= self.speed
            if _key_states[NumKey.NUM4]: lat += self.speed
            if _key_states[NumKey.NUM6]: lat -= self.speed
            if _key_states[NumKey.NUM7]: yaw += self.speed
            if _key_states[NumKey.NUM9]: yaw -= self.speed
        return fwd, lat, yaw

    def stop(self):
        """Gracefully stop the hook thread."""
        if _thread_id:
            ctypes.windll.user32.PostThreadMessageW(_thread_id, _WM_QUIT, 0, 0)
