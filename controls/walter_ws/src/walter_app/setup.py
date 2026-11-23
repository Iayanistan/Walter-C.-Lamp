from setuptools import find_packages, setup

package_name = "walter_app"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["static/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Walter C. Lamp team",
    maintainer_email="noreply@example.com",
    description="Browser-based procedure builder for the Walter cell.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["server = walter_app.server:main"]},
)
