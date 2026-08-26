from pathlib import Path

from setuptools import setup


package_name = "franka_teach_tools"
launch_files = sorted(
    str(path)
    for pattern in ("*.launch.py", "launch/*.launch.py")
    for path in Path(".").glob(pattern)
    if path.is_file()
)

data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml", "README.md"]),
]

if launch_files:
    data_files.append((f"share/{package_name}/launch", launch_files))


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Local User",
    maintainer_email="user@example.com",
    description="Simple script-based teaching tools for Franka point and sequence capture.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teach_cli = franka_teach_tools.teach_cli:main",
            "executor_cli = franka_teach_tools.executor_cli:main",
            "verify_perception = franka_teach_tools.verify_perception:main",
        ],
    },
)
