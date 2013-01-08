from distutils.core import setup
from distutils.command.install_data import install_data


import glob
import os

import make_deb

setup (name = "python-ur", 
        version = make_deb.bzrstring,
        description = "Python library to control an UR robot",
        author = "Olivier Roulet-Dubonnet",
        url = 'http://launchpad.net/XXX',
        py_modules=["urparser", "urx"],
        license = "GNU General Public License",
        )


