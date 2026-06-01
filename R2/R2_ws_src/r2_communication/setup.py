from setuptools import find_packages, setup

package_name = "r2_communication"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/keyboard_control.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="R2 Team",
    maintainer_email="user@example.com",
    description="R2 机器人上下位机 MAVLink 串口通信包，含键盘遥控功能",
    license="MIT",
    entry_points={
        "console_scripts": [
            "r2_keyboard_control = r2_communication.keyboard_control:main",
        ],
    },
)
