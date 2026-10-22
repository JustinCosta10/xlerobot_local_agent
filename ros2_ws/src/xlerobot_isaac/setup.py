from glob import glob

from setuptools import find_packages, setup

setup(
    name="xlerobot_isaac",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/xlerobot_isaac"]),
        ("share/xlerobot_isaac", ["package.xml"]),
        ("share/xlerobot_isaac/scripts", ["scripts/run_isaac.py", "scripts/run_isaac.sh"]),
        ("share/xlerobot_isaac/textures", glob("textures/*")),
        ("share/xlerobot_isaac/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Justin",
    maintainer_email="justin@example.com",
    description="XLeRobot Isaac Sim integration",
    license="Proprietary",
    tests_require=["pytest"],
)
