from distutils.core import setup
from distutils.command.install_data import install_data


import make_deb

setup (name = "python-urx", 
        version = make_deb.vcsstring,
        description = "Python library to control an UR robot",
        author = "Olivier Roulet-Dubonnet",
        url = 'https://github.com/oroulet/python-urx',
        packages = ["urx"],
        provides = ["urx"],
        license = "GNU General Public License v3",
        )


