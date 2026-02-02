from setuptools import find_packages, setup

package_name = "four_dof_arm_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/keyboard_teleop.launch.py",
                "launch/keyboard_servo_teleop.launch.py",
                "launch/cartesian_teleop.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Eli Griffin",
    maintainer_email="eli-griffin@example.com",
    description="Keyboard-to-joy and joy-to-trajectory teleop for the four_dof_arm.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_to_joy = four_dof_arm_teleop.keyboard_to_joy:main",
            "joy_to_trajectory = four_dof_arm_teleop.joy_to_trajectory:main",
            "joy_to_twist = four_dof_arm_teleop.joy_to_twist:main",
            "joy_to_pose_ik = four_dof_arm_teleop.joy_to_pose_ik:main",
        ],
    },
)
