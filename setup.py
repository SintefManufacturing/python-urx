from distutils.core import setup
from distutils.command.install_data import install_data


setup (name = "urx", 
        version = "0.10.0",
        description = "Python library to control an UR robot",
        author = "Olivier Roulet-Dubonnet",
        author_email = "olivier.roulet@gmail.com",
        url = 'https://github.com/oroulet/python-urx',
        packages = ["urx"],
        provides = ["urx"],
        install_requires=["math3d"],
        license = "GNU General Public License v3",

        classifiers = [
            "Programming Language :: Python",
            "Programming Language :: Python :: 3",
            "Development Status :: 4 - Beta",
            "Intended Audience :: Developers",
            "Operating System :: OS Independent",
            "Topic :: System :: Hardware :: Hardware Drivers",
            "Topic :: Software Development :: Libraries :: Python Modules",
        ]
        )


