from glob import glob
from setuptools import find_packages, setup

package_name = "nav_planning_console"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jetson-nano-x1",
    maintainer_email="saturnvdt@gmail.com",
    description="Standalone ROS 2 web console for Nav2 map planning and navigation.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "nav_planning_console = nav_planning_console.node:main",
        ],
    },
)
