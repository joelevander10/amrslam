from setuptools import setup

package_name = "amr_base"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gvipc",
    maintainer_email="igp.indi.4.0.00@gmail.com",
    description="Layer-1 base nodes for the SLAM AMR.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_mux_kinematics_node = amr_base.cmd_mux_kinematics_node:main",
            "diff_drive_odom_node = amr_base.diff_drive_odom_node:main",
            "drive_node = amr_base.drive_node:main",
            "panel_node = amr_base.panel_node:main",
            "commissioning_node = amr_base.commissioning_node:main",
            "qr_base_node = amr_base.qr_base_node:main",
            "qr_calibrate = amr_base.qr_calibrate:main",
        ],
    },
)
