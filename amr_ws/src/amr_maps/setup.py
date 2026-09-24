from glob import glob

from setuptools import setup

package_name = "amr_maps"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds/sim_factory", glob("worlds/sim_factory/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gvipc",
    maintainer_email="igp.indi.4.0.00@gmail.com",
    description="Map artefacts and occupancy-grid file helpers.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["generate_sim_factory = amr_maps.generate_sim_factory:main"]},
)
