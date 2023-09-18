from setuptools import setup, find_packages

setup(
    name='mtlsp',
    version='2.0',
    author='Haowei Sun, Haojie Zhu, Shuo Feng, Yuanxin Zhong',
    author_email='haoweis@umich.edu, zhuhj@umich.edu, fshuo@umich.edu, zyxin@umich.edu',
    packages=["mtlsp", "mtlsp.envs", "mtlsp.logger", "mtlsp.measure", "mtlsp.network", "mtlsp.vehicle", "mtlsp.vehicle.controllers", "mtlsp.vehicle.decision_models", "mtlsp.vehicle.factories", "mtlsp.vehicle.sensors", "mtlsp.physics"],
    scripts=[],
    url='https://github.com/michigan-traffic-lab/MTL-Simulation-Platform',
    license='MIT',
    description='A SUMO-based environment for CAV simulation and evaluation',
    long_description=open('README.md').read(),
    install_requires=['numpy', "bidict", "attrs", "addict", "scipy"],
)