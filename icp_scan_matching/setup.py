from setuptools import find_packages, setup

package_name = "icp_scan_matching"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/icp_launch.py"]),
        (f"share/{package_name}/assets", ["assets/icp_synthetic_demo.png"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="Luis Alvarez",
    maintainer_email="luiangalvgil2@gmail.com",
    description="2D ICP scan matching and meta-scan publisher for AMS Exercise 9.",
    license="Academic coursework",
    entry_points={
        "console_scripts": [
            "icp_node = icp_scan_matching.icp_node:main",
        ],
    },
)

