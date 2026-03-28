from setuptools import setup

package_name = "hardware_controllers"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anthony Botticchio",
    maintainer_email="anthonyjb5228@gmail.com",
    description="QADT hardware controllers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gimbal_controller = hardware_controllers.gimbal_controller:main",
        ],
    },
)