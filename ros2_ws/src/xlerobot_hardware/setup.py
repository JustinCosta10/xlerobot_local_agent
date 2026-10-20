from setuptools import find_packages, setup
setup(name="xlerobot_hardware", version="0.2.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/xlerobot_hardware"]),
                  ("share/xlerobot_hardware", ["package.xml"]),
                  ("share/xlerobot_hardware/config", ["config/hardware.example.yaml"])],
      install_requires=["setuptools", "feetech-servo-sdk==1.0.0"], tests_require=["pytest"],
      entry_points={"console_scripts": ["servo_bridge = xlerobot_hardware.node:main"]})
