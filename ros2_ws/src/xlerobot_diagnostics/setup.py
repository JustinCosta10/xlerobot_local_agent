from setuptools import find_packages, setup
setup(name="xlerobot_diagnostics", version="0.2.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/xlerobot_diagnostics"]),
                  ("share/xlerobot_diagnostics", ["package.xml"])],
      install_requires=["setuptools"], tests_require=["pytest"],
      entry_points={"console_scripts": [
          "bridge_monitor = xlerobot_diagnostics.monitor:main",
          "check_bridge = xlerobot_diagnostics.monitor:check_main"]})
