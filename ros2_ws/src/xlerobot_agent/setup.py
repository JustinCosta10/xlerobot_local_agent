from glob import glob
from setuptools import find_packages, setup

setup(
    name="xlerobot_agent", version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/xlerobot_agent"]),
        ("share/xlerobot_agent", ["package.xml", "README.md"]),
        ("share/xlerobot_agent/config", glob("config/*.yaml")),
        ("share/xlerobot_agent/examples", glob("examples/*.jsonl")),
    ],
    install_requires=["setuptools", "PyYAML", "jsonschema"], zip_safe=True,
    maintainer="Justin", maintainer_email="justin@example.com",
    description="Terminal agent for confirmed-object queries and YOLOE vocabulary control",
    license="Proprietary", tests_require=["pytest"],
    entry_points={"console_scripts": ["agent = xlerobot_agent.cli:main"]},
)
