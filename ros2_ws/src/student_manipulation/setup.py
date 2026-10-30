from setuptools import setup

setup(
    name='student_manipulation', version='0.0.1', packages=['student_manipulation'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/student_manipulation']),
                ('share/student_manipulation', ['package.xml'])],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['task = student_manipulation.task:main']},
)
