from setuptools import setup

package_name = 'wpr_simulation2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='zhangwanjie@126.com',
    description='simulator for 6-robot',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'face_detector = wpr_simulation2.face_detector:main',
        ],
    },
)
