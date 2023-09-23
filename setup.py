try:
    from skbuild import setup
except ImportError:
    raise Exception

setup(
    name="diplan",
    version="0.0.1",
    description="double integrator planner",
    author="Hirokazu Ishida",
    license="MIT",
    packages=["diplan"],
    package_dir={"": "python"},
    cmake_install_dir="python/diplan/",
    package_data={"diplan": ["py.typed"]},
)
