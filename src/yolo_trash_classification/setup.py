from setuptools import setup

package_name = 'yolo_trash_classification'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={
        '': ['package.xml'],
    },
    install_requires=[
        'setuptools',
        'opencv-python',
        'torch',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='YOLOv5 Trash Classification',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_subscriber = yolo_trash_classification.yolo_subscriber:main',
        ],
    },
)