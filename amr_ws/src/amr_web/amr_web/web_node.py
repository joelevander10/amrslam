"""web_node: Flask in the main thread, the rclpy adapter spinning behind it."""

from __future__ import annotations

import os

import rclpy

from amr_web.adapter import RosAdapter, start_spinning
from amr_web.server import create_app


def main(args=None) -> None:
    rclpy.init(args=args)
    adapter = RosAdapter()
    adapter.declare_parameter("maps_dir", os.path.expanduser("~/amr_maps"))
    adapter.declare_parameter("host", "0.0.0.0")
    adapter.declare_parameter("port", 5001)
    adapter.declare_parameter("wifi_iface", "wlp1s0")
    # the commissioning node's evidence directory: same source as base.launch.py uses for it
    adapter.declare_parameter("state_dir", os.environ.get("AMR_STATE_DIR", os.path.expanduser("~/.amr")))
    maps_dir = adapter.get_parameter("maps_dir").value
    host, port = adapter.get_parameter("host").value, int(adapter.get_parameter("port").value)
    spinner = start_spinning(adapter)
    app = create_app(
        adapter,
        maps_dir,
        wifi_iface=str(adapter.get_parameter("wifi_iface").value),
        state_dir=str(adapter.get_parameter("state_dir").value),
    )
    adapter.get_logger().info(f"operator pages on http://{host}:{port}/  (maps_dir {maps_dir})")
    try:
        app.run(host=host, port=port, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        spinner.stop()  # explicit executor shutdown + join before the context goes away
        try:
            adapter.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
