from setuptools import setup, find_packages

package_name = 'task04_basic_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Basic publisher and subscriber example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = task04_basic_pubsub.publisher:main',
            'subscriber = task04_basic_pubsub.subscriber:main',
        ],
    },
)
