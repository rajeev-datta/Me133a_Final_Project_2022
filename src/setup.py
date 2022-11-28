from setuptools import setup
from glob import glob

package_name = 'hw7code'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models/tetherball', glob('models/tetherball/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a HW7 Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw6p2sol = hw7code.hw6p2sol:main',
            'hw6p3sol = hw7code.hw6p3sol:main',
            'hw6p4sol = hw7code.hw6p4sol:main',
            'hw6p5sol = hw7code.hw6p5sol:main',
            'KinematicChainGravity = hw7code.KinematicChainGravity:main',
        ],
    },
)
