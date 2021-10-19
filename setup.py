import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="drone_controller", # Replace with your own username
    version="0.0.1",
    author="Kenny Chour",
    author_email="ckennyc@tamu.edu",
    description="High level olympe interface for controlling parrot drones",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Kchour/drone_controller",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)