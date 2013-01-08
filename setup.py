from distutils.core import setup
from distutils.command.install_data import install_data


import make_deb

setup (name = "python-ur", 
        version = make_deb.vcsstring,
        description = "Python library to control an UR robot",
        author = "Olivier Roulet-Dubonnet",
        url = 'http://launchpad.net/XXX',
        packages = ["urx"],
        provides = ["urx"],
        license = "GNU General Public License v3",
        )


