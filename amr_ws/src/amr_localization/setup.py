from glob import glob

from setuptools import setup

package_name = "amr_localization"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gvipc",
    maintainer_email="igp.indi.4.0.00@gmail.com",
    description="State estimation for the SLAM AMR.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_bias_node = amr_localization.imu_bias_node:main",
            "localization_monitor_node = amr_localization.localization_monitor_node:main",
            "scan_gate_node = amr_localization.scan_gate_node:main",
            "scan_mask_learn = amr_localization.scan_mask_learn:main",
        ],
    },
)
