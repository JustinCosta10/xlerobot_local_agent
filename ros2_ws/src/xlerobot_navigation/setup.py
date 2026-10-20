from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

setup(
    name="xlerobot_navigation", version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/xlerobot_navigation"]),
        ("share/xlerobot_navigation", ["package.xml"]),
        *[(f"share/xlerobot_navigation/{folder}", [p for p in glob(f"{folder}/*") if Path(p).is_file()])
          for folder in ("launch", "config", "rviz", "behavior_trees")],
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Justin", maintainer_email="justin@example.com",
    description="cuVSLAM, Nav2, and nvblox simulation navigation", license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "sim_localization = xlerobot_navigation.sim_localization:main",
        "vslam_localization = xlerobot_navigation.vslam_localization:main",
        "cuvslam_depth_conversion = xlerobot_navigation.depth_conversion:main",
        "navigation_guard = xlerobot_navigation.guard:main",
        "wait_for_mapping = xlerobot_navigation.guard:wait_for_mapping",
    ]},
)
