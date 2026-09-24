"""Q12: a live pose/scan carries the age of the transform it was read from, not the read time."""

from amr_web.live import LiveStore


def test_pose_and_scan_age_include_the_source_age():
    st = LiveStore()
    st.set_pose(1.0, 2.0, 0.0, "map", source_age_s=5.0)
    st.set_scan([(1.0, 1.0)], "map", source_age_s=3.0)
    d = st.pose_scan()
    assert d["pose"]["age_s"] >= 5.0 and d["scan"]["age_s"] >= 3.0
    # re-reading the same frozen transform 200 ms later reports it older, never fresher
    st.set_pose(1.0, 2.0, 0.0, "map", source_age_s=5.2)
    assert st.pose_scan()["pose"]["age_s"] >= 5.2
    # a future-stamped source (AMCL) is clamped to fresh, not negative
    st.set_pose(1.0, 2.0, 0.0, "map", source_age_s=-0.3)
    assert 0.0 <= st.pose_scan()["pose"]["age_s"] < 0.1
