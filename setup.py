from setuptools import find_packages, setup

package_name = "sandbox"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chopgood",
    maintainer_email="chopgood@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "node1 = sandbox.first_node:main",
            "node2 = sandbox.second_node:main",
            "news_pub = sandbox.news_publisher:main",
            "news_sub = sandbox.news_subscriber:main",
            "number_pub = sandbox.number_publisher:main",
            "number_counter = sandbox.number_counter:main",
            "sum2_server = sandbox.sum2_server:main",
        ],
    },
)
