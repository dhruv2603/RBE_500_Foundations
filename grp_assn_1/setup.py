from setuptools import find_packages, setup

package_name = 'grp_assn_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy','scipy'],
    zip_safe=True,
    maintainer='dhruv',
    maintainer_email='cooldhruvagrawal@gmail.com',
    description='Publish Pose by calculating Forward Kinematics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fwd_kin_exec = grp_assn_1.fwd_kn:main'
        ],
    },
)
