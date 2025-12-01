from setuptools import setup, find_packages

setup(
    name="hexapod",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=["matplotlib>=3.10.1"],
)
