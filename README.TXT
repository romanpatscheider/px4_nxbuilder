A simple build skeleton for multiple NuttX configurations.

This skeleton assumes that you have tools installed that suit the configuration you
are building; it will not invoke the buildroot.

Make a directory or symlink called 'dist' and either unpack the NuttX distribution
inside it, or point it to the top-level directory where you have checked out NuttX.

Your layout should look like this:

nxbuilder
  build
  dist
    apps
      ...
    nuttx
      ...
  etc
  <target dir>
  <target dir>
  ...

Note that the build directory will be created automatically as needed; it may not
be present when first you check this project out.

Detail steps (as of Feb. 2012):

0) Download and install either this toolchain or Windows GCC
https://github.com/MikeSmith/summon-arm-toolchain
http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/

1) Download: http://sourceforge.net/projects/nuttx/files/nuttx/nuttx-6.14/nuttx-6.14.tar.gz/download
2) Download: http://sourceforge.net/projects/nuttx/files/nuttx/nuttx-6.14/apps-6.14.tar.gz/download
3) Extract both files
4) Create the directory px_nxbuilder/dist
5) Copy nuttx-6.14 to px_nxbuilder/dist/nuttx
6) Copy apps-6.14 to px_nxbuilder/dist/apps
7) To build for the PX4FMU board, go to the directory (cd px_nxbuilder/px4fmu) and enter 'make'