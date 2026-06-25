from glob import glob

from setuptools import find_packages, setup

package_name = "ur_arm_manager"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hannes Voss",
    maintainer_email="hannes.voss@haw-hamburg.de",
    description="UR5-Manager: einsatzbereit machen + Recovery nach Safety-Violation.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_manager = ur_arm_manager.arm_manager:main",
        ],
    },
)
