from setuptools import setup

package_name = 'dolly'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/dolly.urdf']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/meshes', ['meshes/Lowpolyhuman.dae']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Beraa',
    maintainer_email='beraa@example.com',
    description='Dolly robot model package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

