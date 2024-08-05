from setuptools import find_packages, setup

package_name = 'limo_learnning_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agilex',
    maintainer_email='agilex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "limo_topic_cmd = limo_learnning_py.limo_topic_cmd:main",
            "limo_server = limo_learnning_py.limo_server:main",
            "limo_client = limo_learnning_py.limo_client:main",
            "limo_action_server = limo_learnning_py.limo_action_server:main",
            "limo_action_client = limo_learnning_py.limo_action_client:main"
        ],
    },
)


