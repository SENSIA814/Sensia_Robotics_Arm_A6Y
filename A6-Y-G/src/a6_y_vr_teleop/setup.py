from glob import glob
from setuptools import setup

setup(
    name='a6_y_vr_teleop', version='0.1.0',
    packages=['a6_y_vr_teleop'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/a6_y_vr_teleop']),
        ('share/a6_y_vr_teleop', ['package.xml']),
        ('share/a6_y_vr_teleop/config', glob('config/*')),
        ('share/a6_y_vr_teleop/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='A6-Y-G maintainers', maintainer_email='maintainer@example.invalid',
    description='A5 VR visualization and guarded joint-impedance reference bridge',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['virtual_arm = a6_y_vr_teleop.node:main', 'real_bridge = a6_y_vr_teleop.real_node:main']},
)
