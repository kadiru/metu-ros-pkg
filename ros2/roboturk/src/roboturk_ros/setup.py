from pathlib import Path
from setuptools import find_packages, setup

files = [
    ("share/ament_index/resource_index/packages", ["resource/roboturk_ros"]),
    ("share/roboturk_ros", ["package.xml"]),
]
for directory in ("launch", "webots"):
    parents = {}
    for path in Path(directory).rglob("*"):
        if path.is_file() and "__pycache__" not in path.parts:
            parents.setdefault(str(Path("share/roboturk_ros") / path.parent), []).append(str(path))
    files.extend(parents.items())
setup(
    name="roboturk_ros",
    version="0.1.0",
    packages=find_packages(),
    data_files=files,
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "ssl_referee=roboturk_ros.ssl_referee:main",
            "strategy=roboturk_ros.strategy_node:main",
            "practice_referee=roboturk_ros.practice_referee:main",
        ]
    },
)
