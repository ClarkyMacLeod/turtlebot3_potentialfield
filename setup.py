from setuptools import setup
 
package_name = 'turtlebot3_potentialfield'
 
setup(
    name=package_name,
    version='0.7.3',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Steffen Fleischmann',
    author_email='fleischy_3@web.de',
    maintainer='Steffen Fleischmann',
    maintainer_email='fleischy_3@web.de',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A ROS2 python package to realize a potencialfield',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'potential = turtlebot3_potentialfield.potentialfield:main'
        ],
    },
)
