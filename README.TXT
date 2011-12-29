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

