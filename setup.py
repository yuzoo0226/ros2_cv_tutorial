from setuptools import find_packages, setup

package_name = 'ros2_cv_tutorial'

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
    maintainer='tamhome',
    maintainer_email='yano.yuuga158@mail.kyutech.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = ros2_cv_tutorial.image_publisher:main',
            'image_subscriber = ros2_cv_tutorial.image_subscriber:main',
            'image_extractor_node = ros2_cv_tutorial.image_extractor_node:main',
        ],
    },
)
