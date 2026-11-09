from glob import glob
from setuptools import setup

setup(
    name="turtlebot4_slam_nav2",
    version="0.2.0",
    packages=["turtlebot4_slam_nav2"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/turtlebot4_slam_nav2"]),
        ("share/turtlebot4_slam_nav2", ["package.xml"]),
        *[("share/turtlebot4_slam_nav2/" + name, glob(name + "/*"))
          for name in ("launch", "config", "description", "worlds")],
    ],
    install_requires=["setuptools"],
    maintainer="Luc Van Khoa",
    maintainer_email="khoa@example.com",
    description="TurtleBot 4 simulation support for the Week 7 assignment",
    license="Apache-2.0",
)
