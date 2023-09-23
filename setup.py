try:
    from skbuild import setup
except ImportError:
    raise Exception

setup(
    name="disbmp",
    version="0.0.1",
    description="double integrator planner",
    author="Hirokazu Ishida",
    license="MIT",
    packages=["disbmp"],
    package_dir={"": "python"},
    cmake_install_dir="python/disbmp/",
    package_data={"disbmp": ["py.typed"]},
)
