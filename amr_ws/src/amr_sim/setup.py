from setuptools import setup

package_name = "amr_sim"

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
    description="Sim / mock layer for the SLAM AMR.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_base_node = amr_sim.fake_base_node:main",
            "fake_imu_node = amr_sim.fake_imu_node:main",
            "scan_synth_node = amr_sim.scan_synth_node:main",
            "fake_panel_node = amr_sim.fake_panel_node:main",
            "square_drive = amr_sim.square_drive:main",
        ],
    },
)
