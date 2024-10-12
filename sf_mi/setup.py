from setuptools import setup

package_name = 'sf_mi'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','flask','rclpy','sf_msgs'],
    zip_safe=True,
    maintainer='parth_0531',
    maintainer_email='par6224s@hs-coburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mobile_interface = sf_mi.sf_mi:main"
        ],
    },
)
