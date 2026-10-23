from glob import glob
from setuptools import find_packages, setup

setup(
    name="xlerobot_perception", version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/xlerobot_perception"]),
        ("share/xlerobot_perception", ["package.xml"]),
        ("share/xlerobot_perception/launch", glob("launch/*.launch.py")),
        ("share/xlerobot_perception/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Justin", maintainer_email="justin@example.com",
    description="YOLOE object segmentation and ROS detection feeds", license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "yoloe_node = xlerobot_perception.detector:main",
        "yoloe_visualizer = xlerobot_perception.visualizer:main",
        "object_mapper = xlerobot_perception.object_mapper:main",
    ]},
)
