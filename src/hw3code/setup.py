from setuptools import setup

package_name = 'hw3code'

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
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a HW3 Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw3p3 = hw3code.hw3p3:main',
            'hw3p4 = hw3code.hw3p4:main',
            'hw3p5 = hw3code.hw3p5:main',
        ],
    },
)
