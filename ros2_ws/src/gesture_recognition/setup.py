from glob import glob
from setuptools import find_packages, setup

package_name = "gesture_recognition"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models", glob("models/*.task")),
    ],
    install_requires=["setuptools", "mediapipe"],
    zip_safe=True,
    maintainer="jetson-nano-x1",
    maintainer_email="saturnvdt@gmail.com",
    description="ROS 2 MediaPipe gesture recognition node for the ZED color feed",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "gesture_recognition = gesture_recognition.gesture_recognition_node:main",
        ],
    },
)
