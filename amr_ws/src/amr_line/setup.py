from setuptools import setup

package_name = "amr_line"

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
    description="LINE layer: magnetic-tape following, ported from the gy-demo controller.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "line_follow_node = amr_line.line_follow_node:main",
        ],
    },
)
