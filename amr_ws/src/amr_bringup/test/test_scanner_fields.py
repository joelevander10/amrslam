"""scanner_fields: the field set per vehicle (2026-11-05). gvievo-01's three fields stay as
they were; the AMR QR's scanner has ONE warning field (path 0) and the protective field on
path 1 (read off the scanner: green [T, T], yellow [F, T], red [F, F])."""

import copy
import os
import sys
import types

import pytest
import yaml

from amr_bringup import scanner_fields as sf

CFG = os.path.join(os.path.dirname(__file__), "..", "config")


def _doc(name):
    return yaml.safe_load(open(os.path.join(CFG, name)))


def test_gvievo_keeps_its_three_fields():
    p = sf.mux_params(sf.validate(_doc("scanner_fields.yaml")))
    assert p["protective_index"] == 0 and p["warning_indices"] == [2, 1]
    assert p["warning_scales"] == [0.5, 0.0] and p["warning_stop_m"] == 0.40


def test_the_amr_qr_has_one_warning_field_and_the_protective_on_path_1():
    p = sf.mux_params(sf.validate(_doc("scanner_fields.amr-qr-01.yaml")))
    assert p == {
        "protective_index": 1,
        "warning_indices": [0],
        "warning_scales": [0.5],
        "warning_active_level": False,
        "warning_stop_m": 0.0,
    }


def test_the_qr_readings_map_to_clear_warning_and_stop():
    """The three /output_paths images measured on the vehicle, through the mux's own rules."""
    from amr_base import gating

    fp = gating.FieldParams(**{k: (tuple(v) if isinstance(v, list) else v)
                               for k, v in sf.mux_params(_doc("scanner_fields.amr-qr-01.yaml")).items()})
    measured = (([True, True], True, 0), ([False, True], True, 1), ([False, False], False, 1))
    for status, protective_clear, level in measured:
        v = gating.field_view(1.0, gating.Field(1.0, tuple(status + [False] * 18)), fp)
        assert v.fresh and v.protective_clear == protective_clear and v.warning_level == level, status


def test_a_single_warning_field_cannot_be_a_stop_and_fields_must_nest():
    d = _doc("scanner_fields.amr-qr-01.yaml")
    bad = copy.deepcopy(d)
    bad["fields"]["warning_1"]["scale"], bad["fields"]["warning_1"]["stop_m"] = 0.0, 0.3
    with pytest.raises(ValueError, match="only warning_2"):
        sf.validate(bad)
    bad = copy.deepcopy(d)
    bad["fields"]["warning_1"]["rect"]["x_max"] = 0.3
    with pytest.raises(ValueError, match="nest protective in warning_1"):
        sf.validate(bad)
    bad = copy.deepcopy(d)
    bad["fields"]["warning_1"]["path"] = 1
    with pytest.raises(ValueError, match="own output path"):
        sf.validate(bad)
    bad = copy.deepcopy(d)
    del bad["fields"]["warning_1"]
    with pytest.raises(ValueError, match="expected"):
        sf.validate(bad)


def test_the_vehicle_file_is_picked_by_the_profile(monkeypatch):
    pkg = types.ModuleType("ament_index_python.packages")
    pkg.get_package_share_directory = lambda _name: os.path.join(CFG, "..")
    monkeypatch.setitem(sys.modules, "ament_index_python", types.ModuleType("ament_index_python"))
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", pkg)
    assert sf.default_path("amr-qr-01").endswith("scanner_fields.amr-qr-01.yaml")
    assert sf.default_path("agv-01").endswith(os.sep + "scanner_fields.yaml")
    assert sf.default_path("no-such-vehicle").endswith(os.sep + "scanner_fields.yaml")
