from glob import glob
from setuptools import find_packages, setup

setup(name='rover_navigation', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/rover_navigation']),
                  ('share/rover_navigation', ['package.xml']),
                  ('share/rover_navigation/launch', glob('launch/*.py')),
                  ('share/rover_navigation/config', glob('config/*'))],
      package_data={'rover_navigation': ['*.html']},
      install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
      maintainer='Capstone team', maintainer_email='agfoti04@tamu.edu',
      description='Rover mapping and navigation', license='Apache-2.0',
      entry_points={'console_scripts': [
          'terrain = rover_navigation.terrain_node:main',
          'motion_guard = rover_navigation.guard_node:main',
          'mission = rover_navigation.mission_node:main',
          'demo_mission = rover_navigation.demo_node:main',
          'mission_controls = rover_navigation.mission_controls:main',
          'sample_sites = rover_navigation.sample_sites_node:main',
          'evaluate = rover_navigation.evaluate:main',
          'summarize = rover_navigation.metrics:main']})
