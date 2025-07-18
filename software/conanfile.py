import os
from conan import ConanFile


class FerrariSoftwareConan(ConanFile):
    # ######################################################### #
    #        colcon / ROS 2 specific extension of conan file
    # ######################################################### #

    # load ROS 2 build support
    python_requires = "ros2util/[~8]@3rdparty/stable", "cissy_conantools/[~1]@tools/stable"  # major 8 for use with ROS2 humble
    python_requires_extend = "ros2util.Ros2Base"

    # ######################################################### #
    #                package meta-information
    # ######################################################### #

    # * package name (string) [required]
    #       name of the package
    name = "ferrari_software"

    # * author (string) [required]
    #       current package provider/maintainer
    #       e.g. "Michaela Mustermann <michaela.mustermann@dlr.de>"
    author = "Tommaso Bocchietti <tommaso.bocchietti@dlr.de>"

    # * developers (list) [optional]
    #       additional list of package developers, excluding the 'author'
    #       e.g. ["Michaela Musterfrau <michaela.musterfrau@dlr.de>", "Max Mustermann <max.mustermann@dlr.de>"]
    developers = []

    # * license (string) [recommended]:
    #       please use the "official" licence identifier
    #       you can lookup the identifier on https://spdx.org/licenses/
    #       giving an open source license to a DLR software requires the permission of AL/IL
    #       see also https://wiki.robotic.dlr.de/Open-source_software.
    #       Therefore, the default license is 'proprietary'.
    license = "proprietary"

    # * url (string) [required]
    #       url to internal rmc-github repository
    url = "https://rmc-github.robotic.dlr.de/bocc-to/ferrari_software"

    # * homepage (string) [recommended]
    #       url to project page for containing more documentation
    homepage = "<url of homepage>"

    # * description (string) [recommended]
    description = "<Description of Testproject here>"

    # * topics (tuple of strings) [optional]:
    #       tags that describe the package
    #       e.g. topics = ("<Put some tag here>", "<here>", "<and here>")
    topics = ()

    # * requires (list) [optional]
    #       package dependencies
    #       use conan syntax, e.g. "OtherLib/2.1@otheruser/testing"
    #
    requires = ["ros2_desktop_full/[^8]@3rdparty/stable"]

    # * tool_requires (list) [optional]
    #       build tool requirements, i.e.
    #       all packages that are needed to create the package (e.g. dev tools, compilers,
    #       build systems, code analyzers)
    #
    #       Attention: Those will not be passed downstream. Never attempt to link to a library
    #       coming from a tool requirement, even not a static one. Never read build artifacts
    #       from a tool requirement.
    #
    #       use conan syntax, e.g. "OtherLib/2.1@otheruser/testing"
    tool_requires = []

    # ######################################################### #
    #               build system information
    # ######################################################### #

    # * options (dict of <option key : <value range>) [optional]:
    #       defines options that are made public
    #       e.g. options = {"shared": [True, False]}
    # default: empty
    options = {
        # 'fPIC': [False, True],
        # 'shared': [False, True],
    }

    # * default_options (dict of <option key> ; <default value>) [optional]:
    #       specifies the default value for all options
    #       e.g. default_options = {"shared": False}
    # default: empty
    default_options = {
        # 'fPIC': True,
        # 'shared': True,
    }

    # * settings (tuple of strings) [recommended]:
    #       defines what settings will trigger a package rebuild
    # default: ???
    settings = "os", "compiler", "build_type", "arch"

    def export_sources(self):
        """export function:
        exports all files that are part of the source package
        default: export everything in the directory that is not excluded by .gitignore
        """
        from conan.tools.scm.git import Git
        from conan.tools.files import chdir
        import shutil

        git = Git(self)
        with chdir(self, self.recipe_folder):
            for included_file in git.included_files():
                dst = os.path.join(self.export_sources_folder, included_file)
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                if os.path.isdir(included_file):
                    shutil.copytree(included_file, dst, symlinks=True, dirs_exist_ok=True)
                else:
                    shutil.copy2(included_file, dst)

    # * exports (array of files/pattern) [required]:
    #       defines files that should be exported with the recipe,
    #       e.g. metadata or utility files that should be used by other packages
    #       see https://wiki.robotic.dlr.de/seqm.yaml (at RM) or https://rmc-github.robotic.dlr.de/seth-da/seqmedit
    #       for further details about the default "seqm.yaml"
    exports = ["seqm.yaml"]

    #
    # * no_copy_source (boolean) [optional]
    # by default the collected source files (from export_sources) are
    # copied to a source directory in your conan cache
    # and copied again to the build directory.
    # turning on this option, disables the second copy to the build directory.
    # this can be useful if the sources are quite large and the build system supports
    # a build not in source directory.
    #
    # no_copy_source = True

    # * source_dir (path):
    #       path to sources
    source_dir = "src"


    # * ignore_packages (list) [optional]
    #       list of packages that should not be built
    ignore_packages = []

    # * run_tests (bool) [optional]
    #       execute unit tests
    run_tests = False

    # * skip_tests (str) [optional]
    #       regular expression of tests to skip
    skip_tests = ""

    # * cmake_args (list) [optional]
    #       list of arguments to pass to cmake
    cmake_args = []

    # ######################################################### #
    #            common functions for building and packaging
    # ######################################################### #

    def source(self):
        """
        this function is used for providing and manipulating the source files
        using cissy.conantools, the source is already available in a
        subfolder, defined by 'self.source_dir' (default folder is 'source').
        (see https://docs.conan.io/en/latest/reference/conanfile/methods.html#source)
        """

        pass
