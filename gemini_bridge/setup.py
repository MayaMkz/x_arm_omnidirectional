from setuptools import setup

package_name = 'gemini_bridge'

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
    maintainer='jerry',
    maintainer_email='jerry@todo.todo',
    description='Gemini bridge for ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gemini_bridge_node = gemini_bridge.gemini_bridge_node:main',
            'prompt_display = gemini_bridge.prompt_display:main',
            'gemini_color_bridge = gemini_bridge.gemini_color_bridge:main',
            'pickasso_gui = gemini_bridge.pickasso_gui:main',
            'pickasso_gui_qt = gemini_bridge.pickasso_gui_qt:main',
        ],
    },
)
