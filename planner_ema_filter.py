"""Persistent on/off state for planner EMA output filter (shared across nodes)."""
import os

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
STATE_FILE = os.path.join(PROJECT_DIR, 'planner_ema_filter.state')


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
