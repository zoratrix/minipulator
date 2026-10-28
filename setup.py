from glob import glob

from setuptools import find_packages, setup

package_name = "minipulator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "LICENSE"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", ["urdf/robot.urdf"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/web", glob("web/*.html") + glob("web/*.js")
         + glob("web/*.css")),
        ("share/" + package_name + "/web/vendor",
         ["web/vendor/three.module.js", "web/vendor/LICENSE-three.txt"]),
        ("share/" + package_name + "/web/vendor/addons/controls",
         glob("web/vendor/addons/controls/*.js")),
        ("share/" + package_name + "/firmware", ["firmware/minipulator/minipulator.ino"]),
        ("share/" + package_name + "/udev", ["udev/99-minipulator.rules"]),
        ("share/" + package_name + "/instructions", glob("instructions/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ekaterina Urbakh",
    maintainer_email="ridiculousprince@gmail.com",
    description="ROS 2 driver, MoveIt 2 configuration and web UI for a 6-servo Arduino manipulator.",
    entry_points={
        "console_scripts": [
            "serial_bridge = minipulator.serial_bridge:main",
            "trajectory_executor = minipulator.trajectory_executor:main",
            "demos = minipulator.demos:main",
            "web_ui = minipulator.web_ui:main",
        ],
    },
)
