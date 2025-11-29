from setuptools import find_packages, setup

package_name = 'demo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/resource", ['resource/default.jpg','resource/test1.jpg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='wjz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'python_node = demo_py.python_node:main',
            'person_node = demo_py.person:main',
            'learn_face_detect=demo_py.learn_face_detect:main',
            'face_detect_service = demo_py.face_detect_service:main',
            'face_detect_client = demo_py.face_detect_client:main',
        ],
    },
)
