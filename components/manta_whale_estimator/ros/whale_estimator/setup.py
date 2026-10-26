from setuptools import setup


package_name = "whale_estimator"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_dir={package_name: "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "config.yaml"]),
    ],
    install_requires=["setuptools", "filterpy==1.4.5"],
    zip_safe=True,
    maintainer="odyssey",
    maintainer_email="odyssey@todo.todo",
    description="Per-vehicle whale position and velocity estimator",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "whale_estimator_node = whale_estimator.whale_estimator_node:main",
        ],
    },
)
