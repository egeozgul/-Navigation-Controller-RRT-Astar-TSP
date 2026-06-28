"""Persistent on/off state for planner EMA output filter (shared across nodes)."""
import os

def _state_file() -> str:
    try:
        from nav_stack.paths import ROOT
        return str(ROOT / 'planner_ema_filter.state')
    except ImportError:
        return os.path.normpath(os.path.join(
            os.path.dirname(__file__), '..', '..', '..', 'planner_ema_filter.state',
        ))

STATE_FILE = _state_file()


def load(create_if_missing=True, default=True):
    if os.path.isfile(STATE_FILE):
        try:
            val = open(STATE_FILE, encoding='utf-8').read().strip().lower()
            return val in ('1', 'true', 'on', 'yes')
        except OSError:
            pass
    if create_if_missing:
        save(default)
    return default


def save(enabled):
    os.makedirs(os.path.dirname(STATE_FILE) or '.', exist_ok=True)
    with open(STATE_FILE, 'w', encoding='utf-8') as f:
        f.write('on\n' if enabled else 'off\n')


def toggle():
    enabled = not load(create_if_missing=True, default=True)
    save(enabled)
    return enabled
