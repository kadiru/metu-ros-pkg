from setuptools import find_packages, setup

setup(
    name="roboturk_core",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/roboturk_core"]),
        ("share/roboturk_core", ["package.xml"]),
    ],
    package_data={"roboturk": ["ssl_protocol/*"]},
    install_requires=["setuptools", "numpy>=1.26,<3", "protobuf>=3.20"],
    extras_require={
        "rl": ["gymnasium>=1,<2", "stable-baselines3>=2.4,<3"],
        "test": ["pytest>=8", "gymnasium>=1,<2"],
    },
    entry_points={"console_scripts": ["roboturk=roboturk.cli:main"]},
)
