"""CPU (2026-10-02): the commissioning node takes panel and gyro only while a job exists."""

from amr_base import commissioning_node as cn


class Fake:
    created: list
    destroyed: list


def node():
    n = cn.CommissioningNode.__new__(cn.CommissioningNode)
    n.created, n.destroyed = [], []
    n.create_subscription = lambda typ, topic, cb, qos: n.created.append(topic) or topic
    n.destroy_subscription = lambda sub: n.destroyed.append(sub)
    n._panel = n._panel_t = n._gyro = n._gyro_t = "stale"
    return n


def test_subscribed_on_a_job_and_dropped_with_its_state_after():
    n = node()
    n._job_inputs(False)
    assert n.created == [], "idle: nothing extra"
    n._job_inputs(True)
    n._job_inputs(True)
    assert n.created == ["/amr/panel_state", "/imu/data"], "once per job, not per tick"
    n._job_inputs(False)
    assert sorted(n.destroyed) == ["/amr/panel_state", "/imu/data"]
    assert n._panel is None and n._gyro_t is None, "an old sample must not look fresh to the next job"
