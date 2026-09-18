from setuptools import setup

package_name = 'excavator_models'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed Nouman',
    maintainer_email='ahmed.nouman@centria.fi',
    description='Excavator model selection (v1/v2) and v2 hydraulic linkage node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'linkage_state_publisher = excavator_models.linkage_state_publisher:main',
            'model_info = excavator_models.model_info:main',
        ],
    },
)
