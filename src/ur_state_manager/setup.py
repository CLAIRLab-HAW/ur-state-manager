from glob import glob

from setuptools import find_packages, setup

package_name = "ur_state_manager"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hannes Voss",
    maintainer_email="hannes.voss@haw-hamburg.de",
    description="UR5-State-Manager: einsatzbereit machen + Recovery nach Safety-Violation.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_manager = ur_state_manager.state_manager:main",
            "controller_mode_manager = ur_state_manager.controller_mode_manager:main",
        ],
    },
)
