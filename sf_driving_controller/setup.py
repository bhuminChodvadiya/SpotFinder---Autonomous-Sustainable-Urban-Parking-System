from setuptools import setup

package_name = 'sf_driving_controller'

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
    maintainer='af',
    maintainer_email='rak5684s@hs-coburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "driving_controller = sf_driving_controller.driving_controller:main",
            "pure_pursuit = sf_driving_controller.pure_pursuit:main",
            "parking = sf_driving_controller.parking_manoeuver:main"
        ],
    },
)
