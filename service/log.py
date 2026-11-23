"""Logging for the journal: one line per event, "component: message", with a level.

Under systemd (stdout goes to the journal, JOURNAL_STREAM is set) each line starts with a syslog
priority prefix such as "<4>", which journald turns into the entry's level. That gives:
    journalctl -u scout-lowpower -f            everything, warnings bold/yellow, errors red
    journalctl -u scout-lowpower -f -p warning only what went wrong
In a terminal the same lines are coloured with ANSI codes instead (never sent to the journal:
journalctl would show them as [blob data]).

Levels:
    error    something failed and the boat can't do its job properly
    warning  something is wrong, but the service is handling it (link down, autopilot lost, retry)
    info     the story of the mission: state changes, commands, plans, waypoints, setpoint changes
    debug    detail, only with --verbose (per-tick state line, upload items)

Components: service, mission, planner, pilot, progress, bridge, ardupilot, starlink, iridium, health.
"""
import os
import sys

_PRIORITY = {"error": 3, "warning": 4, "info": 6, "debug": 7}
_COLOUR = {"error": "\033[1;31m", "warning": "\033[33m", "debug": "\033[2m"}
_RESET = "\033[0m"

JOURNAL = bool(os.environ.get("JOURNAL_STREAM"))
TTY = sys.stdout.isatty()
verbose = False                     # set by --verbose: debug lines are printed


def _emit(level, component, msg):
    if level == "debug" and not verbose:
        return
    line = f"{component}: {msg}".replace("\n", " | ")   # one journal entry per event
    if JOURNAL:
        line = f"<{_PRIORITY[level]}>{line}"
    elif TTY and level in _COLOUR:
        line = f"{_COLOUR[level]}{line}{_RESET}"
    print(line, flush=True)


def error(component, msg):
    _emit("error", component, msg)


def warning(component, msg):
    _emit("warning", component, msg)


def info(component, msg):
    _emit("info", component, msg)


def debug(component, msg):
    _emit("debug", component, msg)


def result(ok, component, msg, fail_level="warning"):
    """Log a command result: '<msg> ok' at info, or '<msg> REFUSED' at fail_level."""
    _emit("info" if ok else fail_level, component, f"{msg} {'ok' if ok else 'REFUSED'}")
