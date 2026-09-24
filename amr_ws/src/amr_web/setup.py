from setuptools import setup

package_name = "amr_web"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_data={package_name: ["templates/*.html", "static/*.css", "static/*.js", "static/fonts/*"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="gvipc",
    maintainer_email="igp.indi.4.0.00@gmail.com",
    description="Operator web app for the SLAM AMR.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["web_node = amr_web.web_node:main"]},
)
