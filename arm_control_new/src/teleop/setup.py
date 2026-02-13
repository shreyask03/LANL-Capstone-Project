from setuptools import find_packages, setup

package_name = "arm_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Eli Griffin",
    maintainer_email="eli-griffin@example.com",
    description="Keyboard-to-joy and joy-to-trajectory teleop for the 4-DOF arm.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_to_joy = arm_teleop.keyboard_to_joy:main",
            "joy_to_trajectory = arm_teleop.joy_to_trajectory:main",
        ],
    },
)
