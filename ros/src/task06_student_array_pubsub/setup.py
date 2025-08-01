from setuptools import setup

package_name = 'student_array_pubsub'

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
    maintainer='student',
    maintainer_email='student@example.com',
    description='Student array publisher and subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'student_array_publisher = student_array_pubsub.student_array_publisher:main',
            'student_array_subscriber = student_array_pubsub.student_array_subscriber:main',
        ],
    },
)
