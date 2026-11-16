from setuptools import find_packages, setup

package_name = 'detector'

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
    maintainer='CSE180 student',
    maintainer_email='student@example.com',
    description='Rate-limited person detection for the CSE180 warehouse project',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector = detector.detector_node:main',
        ],
    },
)
