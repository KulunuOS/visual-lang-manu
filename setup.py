from setuptools import find_packages, setup

package_name = "visual_grasp_manu"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/pipeline.yaml"]),
        (f"share/{package_name}/launch", ["launch/grasp_candidates.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="maintainer@example.com",
    description="Configurable visual grasp candidate generation for ROS 2 RGB-D streams.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_candidate_node = visual_grasp_manu.grasp_candidate_node:main",
        ],
    },
)
