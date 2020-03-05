import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="vinca",
    version="0.0.1",
    author="Sean Yen",
    author_email="zishenyan@gmail.com",
    description="Conda recipe generator for ROS packages",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/seanyen/vinca",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.7',
    entry_points={
        'console_scripts': [
            'vinca = vinca.cli:main',
        ]
    }
)
