from setuptools import setup

package_name = 'image_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/images', ['images/sample_image.jpg']),  # コメントアウト - 実際の画像が必要な場合はコメントを外す
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Image publisher and subscriber using OpenCV and cv_bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = image_pubsub.image_publisher:main',
            'image_subscriber = image_pubsub.image_subscriber:main',
        ],
    },
)
