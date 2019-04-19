from setuptools import setup

setup(
    name='vicon-beltpack-postprocess',
    version='0.1',
    description='''postprocess vicon pose data, vicon sync data, sensor data and prepare
    into either a calibration dataset or MLX recording''',
    install_requires = [
        'argparse',
        'numpy',
        'pyyaml',
        'pyquaternion',
    ],
    packages=[
        'vicon-beltpack-postprocess'
    ],
    package_dir={
        'vicon-beltpack-postprocess': 'vicon-beltpack-postprocess'
    }
)
