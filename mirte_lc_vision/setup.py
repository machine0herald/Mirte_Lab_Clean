from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'mirte_lc_vision'

data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]
    ),
    (
        'share/' + package_name,
        ['package.xml']
    ),
]

model_root = Path("mirte_lc_vision/models/ColourdetectionYOLO26n.pt")

for file in model_root.rglob("*"):
    if file.is_file():
        install_dir = (
            "share/"
            + package_name
            + "/"
            + str(file.parent)
        )

        data_files.append(
            (
                install_dir,
                [str(file)]
            )
        )

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 
                        'open3d',
                        'opencv-python'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='matthewdelannoy527@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_test = mirte_lc_vision.test_vision:main',
            'object_locator = mirte_lc_vision.object_locator2:main',
            'classifier_2d = mirte_lc_vision.classifier_2d:main',
        ],
    },
)
