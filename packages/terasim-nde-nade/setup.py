from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy

# Cython extensions with explicit numpy include path
extensions = [
    Extension(
        "terasim_nde_nade.utils.collision.collision_check_cy",
        ["terasim_nde_nade/utils/collision/collision_check_cy.pyx"],
        include_dirs=[numpy.get_include()],
    ),
    Extension(
        "terasim_nde_nade.utils.geometry.geometry_utils_cy",
        ["terasim_nde_nade/utils/geometry/geometry_utils_cy.pyx"],
        include_dirs=[numpy.get_include()],
    ),
    Extension(
        "terasim_nde_nade.utils.trajectory.trajectory_utils_cy",
        ["terasim_nde_nade/utils/trajectory/trajectory_utils_cy.pyx"],
        include_dirs=[numpy.get_include()],
    ),
]

setup(
    ext_modules=cythonize(
        extensions,
        compiler_directives={'language_level': "3"}
    )
)