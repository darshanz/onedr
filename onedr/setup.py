from setuptools import setup, find_packages

setup(
    name='onedr',
    version='0.1.0-alpha1',
    description='onedr is a python library for drone control using MAVLink.',
    url='https://github.com/darshanz/onedr',
    author='Sudarshan Pant',
    author_email='sudarshan.pant@.ucd.ie',
    license='Apache License 2.0',
    packages=find_packages(),
    install_requires=['pymavlink'],

)