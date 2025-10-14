from setuptools import setup

package_name = 'gemini_cmd_vel'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyo',
    maintainer_email='s21c1135sc@s.chibakoudai.jp',
    description='Gemini-based controller that converts camera images into /cmd_vel commands.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gemini_controller = gemini_cmd_vel.gemini_controller_node:main',
        ],
    },
)
