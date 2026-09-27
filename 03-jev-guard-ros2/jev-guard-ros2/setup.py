from glob import glob

from setuptools import find_packages, setup

package_name = "jev_guard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="imjbassi",
    maintainer_email="jaiveerbassi@yahoo.com",
    description="Low-latency safety watchdog for robot policies using Jev.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "guard_node = jev_guard.guard_node:main",
        ],
    },
)
