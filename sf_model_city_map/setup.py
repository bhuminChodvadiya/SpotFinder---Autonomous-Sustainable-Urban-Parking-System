from setuptools import setup

package_name = 'sf_model_city_map'

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
    maintainer_email='oms8357s@hs-coburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sf_viz_ego = sf_model_city_map.sf_viz_ego:main",
            "sf_viz_cars = sf_model_city_map.sf_viz_cars:main",
        ],
    },
)
