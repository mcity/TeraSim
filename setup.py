from setuptools import setup, find_packages

setup(
    name='terasim',
    version='2.0',
    author='Haowei Sun, Haojie Zhu, Shuo Feng, Yuanxin Zhong',
    author_email='haoweis@umich.edu, zhuhj@umich.edu, fshuo@umich.edu, zyxin@umich.edu',
    packages=[
        "terasim", "terasim.agent", "terasim.envs", "terasim.logger", "terasim.measure", "terasim.network", "terasim.physics",
        "terasim.vehicle", "terasim.vehicle.controllers", "terasim.vehicle.decision_models", "terasim.vehicle.factories", "terasim.vehicle.sensors",
        "terasim.traffic_light", "terasim.traffic_light.controllers", "terasim.traffic_light.decision_models", "terasim.traffic_light.factories", "terasim.traffic_light.sensors",
    ],
    scripts=[],
    url='https://github.com/michigan-traffic-lab/TeraSim',
    license='MIT',
    description='A SUMO-based environment for AV testing and evaluation',
    long_description=open('README.md').read(),
    install_requires=['numpy', "bidict", "attrs", "addict", "scipy", "eclipse-sumo", "traci", "libsumo", "sumolib", "redis", "lxml"],
)