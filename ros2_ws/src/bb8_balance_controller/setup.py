import os
from glob import glob
from setuptools import setup

package_name = "bb8_balance_controller"

data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/config", glob("config/*.yaml")),
    (f"share/{package_name}/launch", glob("launch/*.launch.py")),
]

policy_files = glob(os.path.join("..", "..", "..", "policies", "*.npz"))
if policy_files:
    data_files.append((f"share/{package_name}/policies", policy_files))

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="UCLA X1",
    maintainer_email="uclax1@example.com",
    description="Residual balance controller for the B_Cubed kiwi-drive shell robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "balance_controller = bb8_balance_controller.node:main",
        ],
    },
)
