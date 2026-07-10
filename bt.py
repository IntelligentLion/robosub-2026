#!/usr/bin/env python3
"""Minimal behavior-tree engine for the mission scripts.

No external deps (py_trees isn't installed on the sub) — just the classic
tick model: every node returns SUCCESS, FAILURE or RUNNING each tick.

  * ``Leaf``       — subclass and implement ``on_start()`` / ``update()`` /
                     ``on_end(status)``. ``update()`` is called every tick
                     until it returns a non-RUNNING status.
  * ``Sequence``   — children in order; FAILURE aborts, RUNNING resumes at
                     the same child next tick (memory semantics).
  * ``Selector``   — first child to SUCCEED wins; tries the next child on
                     FAILURE (fallback).
  * ``Condition``  — wraps a zero-arg callable → SUCCESS/FAILURE.
  * ``ForceSuccess`` / ``Timeout`` — decorators.
  * ``run(root)``  — ticks the tree at ``rate_hz`` until it finishes.

The tick rate doubles as the control-loop cadence: leaves that stream
movement commands (e.g. a 10 Hz yaw correction) just send one command per
``update()`` call.
"""

import time
from enum import Enum


class Status(Enum):
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'


class Behaviour:
    def __init__(self, name=None):
        self.name = name or type(self).__name__

    def tick(self) -> Status:
        raise NotImplementedError

    def reset(self):
        pass

    def __repr__(self):
        return f'<{type(self).__name__} {self.name!r}>'


class Leaf(Behaviour):
    """Stateful action: on_start() once, update() per tick, on_end() once."""

    def __init__(self, name=None):
        super().__init__(name)
        self._started = False

    def tick(self) -> Status:
        if not self._started:
            self._started = True
            self.on_start()
        status = self.update()
        if status != Status.RUNNING:
            self._started = False
            self.on_end(status)
        return status

    def reset(self):
        if self._started:
            self._started = False
            self.on_end(Status.FAILURE)

    # override these
    def on_start(self):
        pass

    def update(self) -> Status:
        return Status.SUCCESS

    def on_end(self, status: Status):
        pass


class Condition(Leaf):
    """SUCCESS iff fn() is truthy. Never RUNNING."""

    def __init__(self, name, fn):
        super().__init__(name)
        self._fn = fn

    def update(self) -> Status:
        return Status.SUCCESS if self._fn() else Status.FAILURE


class Composite(Behaviour):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = list(children)
        self._idx = 0

    def reset(self):
        for c in self.children:
            c.reset()
        self._idx = 0


class Sequence(Composite):
    """All children must succeed, in order. Memory: resumes mid-sequence."""

    def tick(self) -> Status:
        while self._idx < len(self.children):
            status = self.children[self._idx].tick()
            if status == Status.RUNNING:
                return Status.RUNNING
            if status == Status.FAILURE:
                self._idx = 0
                return Status.FAILURE
            self._idx += 1
        self._idx = 0
        return Status.SUCCESS


class Selector(Composite):
    """First success wins; falls through to the next child on failure."""

    def tick(self) -> Status:
        while self._idx < len(self.children):
            status = self.children[self._idx].tick()
            if status == Status.RUNNING:
                return Status.RUNNING
            if status == Status.SUCCESS:
                self._idx = 0
                return Status.SUCCESS
            self._idx += 1
        self._idx = 0
        return Status.FAILURE


class Decorator(Behaviour):
    def __init__(self, child, name=None):
        super().__init__(name or f'{type(self).__name__}({child.name})')
        self.child = child

    def reset(self):
        self.child.reset()


class ForceSuccess(Decorator):
    """FAILURE → SUCCESS (mission keeps moving past optional steps)."""

    def tick(self) -> Status:
        status = self.child.tick()
        return Status.RUNNING if status == Status.RUNNING else Status.SUCCESS


class Timeout(Decorator):
    """FAILURE if the child is still RUNNING after `seconds`."""

    def __init__(self, child, seconds, name=None):
        super().__init__(child, name)
        self.seconds = seconds
        self._deadline = None

    def tick(self) -> Status:
        if self._deadline is None:
            self._deadline = time.monotonic() + self.seconds
        status = self.child.tick()
        if status == Status.RUNNING and time.monotonic() > self._deadline:
            print(f'[bt] TIMEOUT: {self.child.name} after {self.seconds:.0f}s')
            self.child.reset()
            status = Status.FAILURE
        if status != Status.RUNNING:
            self._deadline = None
        return status

    def reset(self):
        self._deadline = None
        super().reset()


def run(root, rate_hz=10, on_tick=None):
    """Tick `root` at rate_hz until it returns a non-RUNNING status.

    Returns the final Status. KeyboardInterrupt propagates after resetting
    the tree (so leaves get their on_end() cleanup).
    """
    period = 1.0 / rate_hz
    try:
        while True:
            status = root.tick()
            if on_tick:
                on_tick(status)
            if status != Status.RUNNING:
                print(f'[bt] tree finished: {status.value}')
                return status
            time.sleep(period)
    except (KeyboardInterrupt, Exception):
        root.reset()
        raise
