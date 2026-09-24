from setuptools import setup

package_name = "amr_mission"

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
    description="Coordinators for the SLAM AMR.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mapping_session_node = amr_mission.mapping_session_node:main",
            "world_bundle = amr_mission.fixtures:main",
            "route_executor_node = amr_mission.route_executor_node:main",
            "map_edit = amr_mission.map_edit:main",
            "survey_move_node = amr_mission.survey_move_node:main",
        ],
    },
)
