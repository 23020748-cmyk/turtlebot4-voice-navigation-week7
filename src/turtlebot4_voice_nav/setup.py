from glob import glob
from setuptools import setup

setup(
    name="turtlebot4_voice_nav",
    version="1.0.0",
    packages=["turtlebot4_voice_nav"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/turtlebot4_voice_nav"],
        ),
        ("share/turtlebot4_voice_nav", ["package.xml", "README.md"]),
        (
            "share/turtlebot4_voice_nav/maps",
            glob("maps/*.json")
            + glob("maps/*.png")
            + glob("maps/*.yaml")
            + glob("maps/*.pgm"),
        ),
        ("share/turtlebot4_voice_nav/maps/original", glob("maps/original/*")),
        ("share/turtlebot4_voice_nav/config", glob("config/*.yaml")),
        ("share/turtlebot4_voice_nav/launch", glob("launch/*.launch.py")),
        ("share/turtlebot4_voice_nav/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HRI coursework",
    maintainer_email="maintainer@example.com",
    description="Vietnamese semantic voice navigation on TurtleBot 4 and Nav2",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "voice_navigation=turtlebot4_voice_nav.navigation_node:main",
            "voice_input=turtlebot4_voice_nav.speech:main",
            "voice_dashboard=turtlebot4_voice_nav.dashboard:main",
        ]
    },
)
