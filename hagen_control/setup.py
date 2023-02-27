from setuptools import setup

package_name = 'hagen_control'

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
    maintainer='op',
    maintainer_email='op@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'diff_drive = hagen_control.hagen_control_strategy:main',
                'PID = hagen_control.diff_drive_PID_control:main',
                'LQR = hagen_control.diff_drive_LQR:main',
                'MPC = hagen_control.diff_drive_MPC:main',
        ],
    },
)
