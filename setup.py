from setuptools import find_packages, setup

package_name = "my_pytree_bt"

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
    maintainer="minegearscsu",
    maintainer_email="aprlagare1999@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "my_pytree_bt_node = my_pytree_bt.my_pytree_bt_node:main",
            "pytree_gears_node = my_pytree_bt.pytree_gears_node:main",
            "pytree_gears_node_no_bb = my_pytree_bt.pytree_gears_node_no_bb:main",
            "pytree_real_gears_node = my_pytree_bt.pytree_real_gears_node:main",
            "pytree_real_gears_node_no_bb = my_pytree_bt.pytree_real_gears_node_no_bb:main",
            "pytree_real_gears_final_node = my_pytree_bt.pytree_real_gears_final_node:main",
        ],
    },
)
