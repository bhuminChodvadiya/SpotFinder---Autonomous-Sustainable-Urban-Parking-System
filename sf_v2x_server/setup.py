from setuptools import setup

package_name = 'sf_v2x_server'

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
    maintainer_email='fel1274s@hs-coburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "v2x_server = sf_v2x_server.sf_v2x_server:main",
            "cam_server = sf_v2x_server.sf_cam_server:main"
        ],
    },
)
