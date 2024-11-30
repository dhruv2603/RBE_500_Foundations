from setuptools import find_packages, setup

package_name = 'grp_assn_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhruv',
    maintainer_email='cooldhruvagrawal@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cal_twist = grp_assn_2.joint_vel_srv:main',
            'cal_joint_vel = grp_assn_2.end_eff_vel_srv:main',
            'incremental_pos = grp_assn_2.incremental_pos_client:main',
        ],
    },
)
