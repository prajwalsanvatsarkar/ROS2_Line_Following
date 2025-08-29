from setuptools import find_packages, setup
from glob import glob

package_name = 'my_cv_pakage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='sanvatsarkar.prajwal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_view = my_cv_pakage.cv_view:main',
            'cv_color_detect = my_cv_pakage.cv_color_detect:main',
            'follow_line_node = my_cv_pakage.follow_line_node:main',
        ],
    },
)

