from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arrow_key_teleop_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('lib/' + package_name, [package_name+'/python_file.py']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='samuko',
    maintainer_email='samuel.c.agba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file_name:main'
            'arrow_key_teleop_drive = arrow_key_teleop_drive.arrow_key_teleop_drive:main',
        ],
    },
)
