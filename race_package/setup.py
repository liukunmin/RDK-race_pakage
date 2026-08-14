
from setuptools import setup, find_packages



package_name = 'race_package'



setup(

    name=package_name,

    version='1.0.0',

    packages=find_packages(),

    data_files=[

        ('share/ament_index/resource_index/packages',

            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

    ],


    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='root',

    maintainer_email='root@localhost',

    description='智能汽车竞赛 - 比赛任务控制包',

    license='Apache-2.0',

    entry_points={

        'console_scripts': [

            'main = race_package.main:main',

            'capture_and_publish = race_package.capture_and_publish:main',

            'sign_trigger = race_package.sign_trigger:main',

            'laser_corrector = race_package.laser_corrector:main',

        ],

    },

)

