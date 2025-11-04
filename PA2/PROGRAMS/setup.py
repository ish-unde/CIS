from xml.etree.ElementInclude import include
import setuptools

setuptools.setup(
    name="CISPA2",
    version="0.0.1",
    author="Ishita Unde and Angela Appiah",
    author_email="iunde1@jhu.edu, aappiah7@jh.edu",
    description="cispa2",
    long_description="cispa2",
    packages=setuptools.find_packages(),
    install_requires=[
        "numpy",
        "scipy",
        "click"
    ],
    include_package_data=True,
    python_requires=">=3.9",
)