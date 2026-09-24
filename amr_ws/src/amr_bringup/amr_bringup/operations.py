"""Bounded operation records with request deduplication and a crash journal
(unified plan §3.2, §4.2 OperationState).

    book = OperationBook(path)              # replays the journal: PENDING -> INTERRUPTED
    op, created = book.submit(request_id, kind, ...)
    book.finish(op.operation_id, SUCCEEDED, message, map_id=..., ...)
    book.get(operation_id)

One pending operation at a time: a second, different request while one is
pending is BUSY (the caller decides that via `book.pending`). The same
request_id returns the existing record, so a browser retry after a dropped
response never creates a second operation. The journal is append-only JSON
lines; side effects are never replayed from it.
"""

from __future__ import annotations

import json
import os
import time
import uuid
from collections import OrderedDict
from dataclasses import asdict, dataclass, field

PENDING, SUCCEEDED, FAILED, INTERRUPTED = 0, 1, 2, 3
STATUS_NAMES = {PENDING: "PENDING", SUCCEEDED: "SUCCEEDED", FAILED: "FAILED", INTERRUPTED: "INTERRUPTED"}
HISTORY = 64


@dataclass
class Operation:
    operation_id: str
    request_id: str
    kind: str
    phase: str = ""
    status: int = PENDING
    map_id: str = ""
    map_revision: int = 0
    map_sha256: str = ""
    message: str = ""
    started: float = field(default_factory=time.time)
    finished: float = 0.0

    @property
    def pending(self) -> bool:
        return self.status == PENDING


class OperationBook:
    def __init__(self, journal_path: str | None = None, history: int = HISTORY) -> None:
        self._ops: OrderedDict[str, Operation] = OrderedDict()
        self._by_request: dict[str, str] = {}
        self._history = history
        self._path = journal_path
        self.interrupted: list[Operation] = []
        if journal_path:
            self._replay(journal_path)

    # -- lifecycle --

    @property
    def pending(self) -> Operation | None:
        for op in reversed(self._ops.values()):
            if op.pending:
                return op
        return None

    def submit(self, request_id: str, kind: str, **fields) -> tuple[Operation, bool]:
        """(operation, created). An existing request_id returns its record unchanged."""
        if request_id and request_id in self._by_request:
            return self._ops[self._by_request[request_id]], False
        op = Operation(operation_id=uuid.uuid4().hex[:12], request_id=request_id, kind=kind, **fields)
        self._ops[op.operation_id] = op
        if request_id:
            self._by_request[request_id] = op.operation_id
        self._trim()
        self._journal(op, "submit")
        return op, True

    def phase(self, operation_id: str, phase: str) -> None:
        op = self._ops[operation_id]
        op.phase = phase

    def finish(self, operation_id: str, status: int, message: str = "", **fields) -> Operation:
        op = self._ops[operation_id]
        if not op.pending:
            return op
        for k, v in fields.items():
            setattr(op, k, v)
        op.status, op.message, op.finished = status, message, time.time()
        self._journal(op, "finish")
        return op

    def get(self, operation_id: str) -> Operation | None:
        return self._ops.get(operation_id)

    def recent(self, n: int = 10) -> list[Operation]:
        return list(self._ops.values())[-n:]

    # -- internals --

    def _trim(self) -> None:
        while len(self._ops) > self._history:
            oid, op = next(iter(self._ops.items()))
            if op.pending:
                break
            self._ops.popitem(last=False)
            self._by_request.pop(op.request_id, None)

    def _journal(self, op: Operation, event: str) -> None:
        if not self._path:
            return
        rec = dict(asdict(op), event=event, t=time.time())
        try:
            os.makedirs(os.path.dirname(self._path) or ".", exist_ok=True)
            with open(self._path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
                f.flush()
                os.fsync(f.fileno())
        except OSError:
            pass  # a journal that cannot be written must not stop the vehicle software

    def _replay(self, path: str) -> None:
        """Rebuild terminal history; anything still PENDING was interrupted by a
        crash/restart and is marked so - its side effects are NOT replayed."""
        try:
            with open(path, encoding="utf-8") as f:
                lines = f.readlines()
        except FileNotFoundError:
            return
        for line in lines:
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            rec.pop("event", None)
            rec.pop("t", None)
            try:
                op = Operation(**rec)
            except TypeError:
                continue
            self._ops[op.operation_id] = op
            if op.request_id:
                self._by_request[op.request_id] = op.operation_id
        for op in self._ops.values():
            if op.pending:
                op.status, op.finished = INTERRUPTED, time.time()
                op.message = (op.message + " " if op.message else "") + "interrupted by supervisor restart"
                self.interrupted.append(op)
                self._journal(op, "interrupted")
        self._trim()
