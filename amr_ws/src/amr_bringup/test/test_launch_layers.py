"""U1 gate (unified plan): launch layers are separable and mode layers are clean.

Static: each launch file is loaded and its entity tree walked with a stub
context, without starting processes or a ROS graph. What this proves is
composition - which executables each entry point can spawn - not runtime
behaviour (that is P7's sequential simulation).
"""

from __future__ import annotations

import os

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch_ros.actions import Node

from amr_bringup import domains

LAUNCH = os.path.join(get_package_share_directory("amr_bringup"), "launch")
BASE_EXES = {
    "cmd_mux_kinematics_node",
    "diff_drive_odom_node",
    "imu_bias_node",
    "scan_gate_node",
    "ekf_node",
    "robot_state_publisher",
}
HW_EXES = {"drive_node", "panel_node", "sick_safetyscanners2_node"}
SIM_EXES = {"fake_base_node", "fake_imu_node", "fake_panel_node", "scan_synth_node"}
MAPPING_EXES = {"async_slam_toolbox_node", "mapping_session_node", "survey_move_node"}
NAV_EXES = {
    "map_server",
    "amcl",
    "lifecycle_manager",
    "localization_monitor_node",
    "controller_server",
    "behavior_server",
    "route_executor_node",
}


def _text(x, context) -> str:
    if isinstance(x, str):
        return x
    if hasattr(x, "perform"):
        return x.perform(context)
    return "".join(_text(i, context) for i in x)


def _walk(entities, context, out: dict, depth=0):
    """Collect executables and included files. Conditions are ignored: this is
    'what CAN this launch spawn', which is the stricter question."""
    for e in entities:
        if isinstance(e, DeclareLaunchArgument):
            if e.name not in context.launch_configurations and e.default_value is not None:
                context.launch_configurations[e.name] = "".join(s.perform(context) for s in e.default_value)
            continue
        if isinstance(e, Node):
            exe = e.node_executable
            exe = "".join(s.perform(context) for s in exe) if isinstance(exe, list) else str(exe)
            out.setdefault("exes", []).append(exe)
            out.setdefault("params", {})[exe] = list(e._Node__parameters or [])
            continue
        if isinstance(e, IncludeLaunchDescription):
            raw = e.launch_description_source._LaunchDescriptionSource__location  # unexpanded substitutions
            path = "".join(s.perform(context) for s in raw) if not isinstance(raw, str) else raw
            out.setdefault("includes", []).append(os.path.basename(path))
            sub_ctx = LaunchContext()
            sub_ctx.launch_configurations.update(context.launch_configurations)
            for k, v in e.launch_arguments:
                sub_ctx.launch_configurations[_text(k, context)] = _text(v, context)
            if path.endswith(".py"):
                sub = get_launch_description_from_any_launch_file(path)
                _walk(sub.entities, sub_ctx, out, depth + 1)
            continue
        if isinstance(e, OpaqueFunction):
            fn, a, kw = e._OpaqueFunction__function, e._OpaqueFunction__args, e._OpaqueFunction__kwargs
            produced = fn(context, *a, **kw)
            _walk(produced or [], context, out, depth + 1)
            continue
        if isinstance(e, TimerAction):
            _walk(e.actions, context, out, depth + 1)
            continue
        if isinstance(e, RegisterEventHandler):
            out["handlers"] = out.get("handlers", 0) + 1
            continue
        if isinstance(e, LaunchDescription):
            _walk(e.entities, context, out, depth + 1)


def compose(name: str, **args) -> dict:
    ld = get_launch_description_from_any_launch_file(os.path.join(LAUNCH, name))
    ctx = LaunchContext()
    ctx.launch_configurations.update({k: str(v) for k, v in args.items()})
    out: dict = {}
    _walk(ld.entities, ctx, out)
    out.setdefault("exes", [])
    out.setdefault("includes", [])
    return out


@pytest.fixture(autouse=True)
def sim_domain(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(domains.SIM))


def test_mapping_layer_is_only_slam_and_coordinator():
    c = compose("mapping_layer.launch.py")
    assert set(c["exes"]) == MAPPING_EXES
    assert c["includes"] == []
    # slam and the session are required; survey_move is not. A lifecycle slam_toolbox
    # (after Humble) adds the configured -> activate handler.
    import importlib.util  # noqa: PLC0415

    spec = importlib.util.spec_from_file_location("ml", os.path.join(LAUNCH, "mapping_layer.launch.py"))
    ml = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ml)
    assert c["handlers"] == (3 if ml.slam_is_lifecycle() else 2)


def test_navigation_layer_has_no_base_web_or_sim(tmp_path):
    # needs a verified bundle to compose; use the world fixture
    from amr_mission.fixtures import write_world_as_bundle  # noqa: PLC0415

    write_world_as_bundle(str(tmp_path))
    c = compose("navigation_layer.launch.py", maps_dir=str(tmp_path), map_id="sim_factory")
    assert set(c["exes"]) == NAV_EXES
    assert not (set(c["exes"]) & (BASE_EXES | HW_EXES | SIM_EXES | {"web_node"}))
    assert c["includes"] == []
    assert c["handlers"] == 6  # every node except the two lifecycle managers


def test_scanner_layer_has_no_description():
    c = compose("scanner.launch.py")
    assert c["exes"] == ["sick_safetyscanners2_node"] and c["includes"] == []


def test_sim_base_is_fakes_plus_estimation_and_one_description():
    c = compose("base.launch.py", real="false")
    assert set(c["exes"]) == SIM_EXES | BASE_EXES
    assert c["exes"].count("robot_state_publisher") == 1
    assert not (set(c["exes"]) & (HW_EXES | MAPPING_EXES | NAV_EXES | {"web_node"}))


def test_real_base_composes_hardware_once(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(domains.VEHICLE))
    c = compose("base.launch.py", real="true")
    # fake_panel is conditional (panel:=fake); this walk ignores conditions
    assert set(c["exes"]) == HW_EXES | BASE_EXES | {"fake_panel_node"}
    assert c["exes"].count("robot_state_publisher") == 1  # the scanner include no longer brings a second one
    assert c["includes"].count("scanner.launch.py") == 1
    assert "web_node" not in c["exes"]


def test_wrappers_compose_base_web_and_exactly_one_layer(tmp_path):
    from amr_mission.fixtures import write_world_as_bundle  # noqa: PLC0415

    write_world_as_bundle(str(tmp_path))
    m = compose("mapping.launch.py", sim="true", maps_dir=str(tmp_path))
    n = compose("nav.launch.py", sim="true", maps_dir=str(tmp_path), map_id="sim_factory")
    for c in (m, n):
        assert c["includes"].count("base.launch.py") == 1
        assert c["exes"].count("robot_state_publisher") == 1
        assert c["exes"].count("web_node") == 1
    assert set(m["exes"]) & MAPPING_EXES == MAPPING_EXES and not set(m["exes"]) & NAV_EXES
    assert set(n["exes"]) & NAV_EXES == NAV_EXES and not set(n["exes"]) & MAPPING_EXES


def test_hardware_wrapper_refuses_sim_domain():
    with pytest.raises(RuntimeError, match="vehicle"):
        compose("drivers.launch.py")


def test_q20_custom_footprint_reaches_the_costmap_node_through_a_params_file(tmp_path, monkeypatch):
    """An inline {"local_costmap.local_costmap.footprint": ...} dict would have set a parameter of
    that literal name on controller_server; the costmap node only sees a node-scoped file."""
    import yaml  # noqa: PLC0415
    from amr_mission.fixtures import write_world_as_bundle  # noqa: PLC0415

    write_world_as_bundle(str(tmp_path))
    fp = tmp_path / "fp.yaml"
    fp.write_text(
        "polygon:\n  - [-0.2, -0.1]\n  - [0.7, -0.1]\n  - [0.7, 0.1]\n  - [-0.2, 0.1]\nmargin_m: 0.05\n"
    )
    monkeypatch.setenv("AMR_STATE_DIR", str(tmp_path / "state"))
    c = compose(
        "navigation_layer.launch.py", maps_dir=str(tmp_path), map_id="sim_factory", footprint_yaml=str(fp)
    )
    from launch_ros.parameter_descriptions import ParameterFile  # noqa: PLC0415

    params = c["params"]["controller_server"]
    # no inline dict: nothing can land on the wrong node
    assert all(isinstance(p, ParameterFile) for p in params)
    files = ["".join(t.text for t in p._ParameterFile__param_file) for p in params]
    scoped = [f for f in files if "costmap_footprint" in f]
    assert len(scoped) == 1
    doc = yaml.safe_load(open(scoped[0]))
    assert doc["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"] == (
        "[[-0.200, -0.100], [0.700, -0.100], [0.700, 0.100], [-0.200, 0.100]]"
    )


def _param_dict(p: dict) -> dict:
    """launch_ros (Humble) normalises a parameters dict: keys and values become tuples of
    substitutions, strings YAML-dumped. Perform and load them back, as launch_ros does."""
    import yaml  # noqa: PLC0415

    ctx = LaunchContext()

    def val(v):
        if isinstance(v, (tuple, list)) and v and all(hasattr(i, "perform") for i in v):
            return yaml.safe_load(_text(v, ctx))
        return v

    return {_text(k, ctx): val(v) for k, v in p.items()}


def _bundle_with_dynamic(tmp_path, big=False):
    from amr_mission.fixtures import write_world_as_bundle  # noqa: PLC0415

    from amr_mission import map_bundle as mb  # noqa: PLC0415

    write_world_as_bundle(str(tmp_path))
    # a mark over the aisle and the rack faces at y 2..2.5 (sim world), or over everything
    rect = (
        [[-50.0, -50.0], [50.0, -50.0], [50.0, 50.0], [-50.0, 50.0]]
        if big
        else [[4.0, -1.0], [9.0, -1.0], [9.0, 2.7], [4.0, 2.7]]
    )
    path, rev, _ = mb.derive_edit(str(tmp_path), "sim_factory", 1, [{"op": "dynamic", "polygon": rect}])
    return path, rev


def test_dynamic_mask_reaches_the_monitor_and_amcl_is_plain_by_default(tmp_path, monkeypatch):
    path, rev = _bundle_with_dynamic(tmp_path)
    monkeypatch.setenv("AMR_STATE_DIR", str(tmp_path / "state"))
    c = compose("navigation_layer.launch.py", maps_dir=str(tmp_path), map_id="sim_factory", revision=rev)
    assert set(c["exes"]) == NAV_EXES and c["exes"].count("map_server") == 1
    mon = [_param_dict(p) for p in c["params"]["localization_monitor_node"] if isinstance(p, dict)]
    assert mon[0]["dynamic_yaml"] == os.path.join(path, "dynamic.yaml")
    assert not (tmp_path / "state" / "loc_map_gen0.yaml").exists()


def test_blank_dynamic_serves_amcl_a_derived_map_on_map_loc(tmp_path, monkeypatch):
    import numpy as np  # noqa: PLC0415

    from amr_maps import grid as gridio  # noqa: PLC0415

    path, rev = _bundle_with_dynamic(tmp_path)
    monkeypatch.setenv("AMR_STATE_DIR", str(tmp_path / "state"))
    c = compose(
        "navigation_layer.launch.py",
        maps_dir=str(tmp_path),
        map_id="sim_factory",
        revision=rev,
        blank_dynamic="true",
    )
    assert c["exes"].count("map_server") == 2 and c["handlers"] == 7
    loc = gridio.read(str(tmp_path / "state" / "loc_map_gen0.yaml"))
    ref = gridio.read(os.path.join(path, "map.yaml"))
    dyn = gridio.read(os.path.join(path, "dynamic.yaml")).data >= 65
    assert (ref.data[dyn] >= 65).any()  # the fixture's rack/wall cells inside the mark...
    assert (loc.data[dyn] == -1).all()  # ...are unknown for AMCL
    assert np.array_equal(loc.data[~dyn], ref.data[~dyn])  # and nothing else changed


def test_blank_dynamic_refuses_a_map_with_nothing_fixed_left(tmp_path, monkeypatch):
    _, rev = _bundle_with_dynamic(tmp_path, big=True)
    monkeypatch.setenv("AMR_STATE_DIR", str(tmp_path / "state"))
    with pytest.raises(RuntimeError, match="nothing fixed to localise against"):
        compose(
            "navigation_layer.launch.py",
            maps_dir=str(tmp_path),
            map_id="sim_factory",
            revision=rev,
            blank_dynamic="true",
        )
