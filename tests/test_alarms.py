"""The alarm catalogue is the contract between the nodes and the operator surface.

Two things must hold, or the operator gets a blank panel where a sentence belongs:
every code a node can emit has a row, and the RUNBOOK's generated section matches
the rows. Both are checked by scanning the source, not by a hand-kept list.
"""

import pathlib
import re

import pytest

from agv_core import alarms

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "amr_ws" / "src"

# How a code reaches a message or an event in this stack. Each pattern's group 1 is the code.
CODE_PATTERNS = (
    re.compile(r'_fail_active\(\s*"([A-Z][A-Z0-9_]+)"'),
    re.compile(r'_fail\([^,]+,\s*"([A-Z][A-Z0-9_]+)"'),
    re.compile(r'fault_code\s*=\s*"([A-Z][A-Z0-9_]+)"'),
    re.compile(r'_fault\((?:[^()]|\([^()]*\))*?,\s*"([A-Z][A-Z0-9_]+)"\s*,?\s*\)', re.S),
    re.compile(r'_set\(\s*fsm\.\w+,[^)]*?"([A-Z][A-Z0-9_]{4,})"\s*\)', re.S),
    re.compile(r'code\s*=\s*"([A-Z][A-Z0-9_]+)"'),
    re.compile(r'self\.event\(\s*"([A-Z][A-Z0-9_]+)"'),
    re.compile(r'\.event\(\s*\d+\s*,\s*"([A-Z][A-Z0-9_]+)"'),
    re.compile(r'_event\(\s*Event\.\w+\s*,\s*"([A-Z][A-Z0-9_]+)"'),
    re.compile(r'_edge\(\s*"[a-z]+"\s*,\s*\(\s*"([A-Z][A-Z0-9_]+)"'),
    re.compile(r'_lose\((?:[^()]|\([^()]*\))*?,\s*"(LOC_[A-Z0-9_]+)"', re.S),
)
# Literals these patterns can also catch that are not alarm codes.
NOT_CODES = {"PENDING", "FAILED", "SUCCEEDED", "INTERRUPTED"}


def _codes_in_source() -> dict[str, set[str]]:
    found: dict[str, set[str]] = {}
    for path in sorted(SRC.rglob("*.py")):
        if "/test/" in str(path) or path.name.startswith("test_"):
            continue
        text = path.read_text()
        for pattern in CODE_PATTERNS:
            for code in pattern.findall(text):
                if code in NOT_CODES:
                    continue
                found.setdefault(code, set()).add(str(path.relative_to(ROOT)))
    return found


def test_every_code_emitted_by_a_node_has_a_catalogue_row():
    """A code with no row reaches the operator as UNKNOWN: 'call the engineer'."""
    missing = {c: sorted(w) for c, w in _codes_in_source().items() if c not in alarms.CATALOGUE}
    assert not missing, f"codes emitted with no catalogue row: {missing}"


def test_the_catalogue_is_well_formed():
    for code, row in alarms.CATALOGUE.items():
        assert row.code == code
        assert row.severity in (alarms.INFO, alarms.WARN, alarms.ERROR), code
        assert row.clears_by in alarms.CLEARS, code
        # The operator's two sentences: a state, then one imperative. Both mandatory.
        assert row.title and row.title[0].isupper(), code
        assert row.action.endswith("."), code
        assert row.hint, code
        assert alarms.BUTTON[row.clears_by] is not None, code


def test_hold_causes_all_map_to_a_catalogue_code():
    """RunState.hold_cause and LineState.hold_cause share this vocabulary."""
    for cause, code in alarms.HOLD_CODES.items():
        assert code in alarms.CATALOGUE, cause


def test_runbook_section_4_matches_the_catalogue():
    """The paper and the screen are generated from the same rows (plan phase 4)."""
    text = (ROOT / "amr_ws" / "RUNBOOK.md").read_text()
    begin = "<!-- BEGIN GENERATED: python3 -m agv_core.alarms --md -->\n"
    end = "\n<!-- END GENERATED -->"
    if begin not in text:
        pytest.fail("RUNBOOK section 4 has lost its generated block")
    block = text.split(begin, 1)[1].split(end, 1)[0]
    assert block == alarms.markdown(), (
        "the catalogue changed: run python3 -m agv_core.alarms --md and paste into RUNBOOK section 4"
    )


def test_the_operator_card_is_short_and_real():
    card = alarms.card()
    assert len(card.splitlines()) <= 20
    for code in alarms.CARD_CODES:
        assert code in alarms.CATALOGUE
        assert alarms.CATALOGUE[code].title in card
