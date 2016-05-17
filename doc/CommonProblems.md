# Common problems

    ../lib/lsd_core/liblsdslam.so: undefined reference to `g2o::csparse_extension::cs_chol_workspace(cs_di_sparse const*, cs_di_symbolic const*, int*, double*)'
    ../lib/lsd_core/liblsdslam.so: undefined reference to `g2o::csparse_extension::cs_cholsolsymb(cs_di_sparse const*, double*, cs_di_symbolic const*, double*, int*)'
    ../lib/lsd_core/liblsdslam.so: undefined reference to `g2o::csparse_extension::writeCs2Octave(char const*, cs_di_sparse const*, bool)'

g2o should be built with the system libcsparse provided by the libsuitesparse-dev package.  Ensure the CMake variable  BUILD_CSPARSE=OFF, and that CSPARSE_INCLUDE_DIR and CSPARSE_LIBRARY point to system libraries, not the libraries included in the g2o source code.

    ../lib/lsd_core/liblsdslam.so: undefined reference t to `pangolin::CreateGlutWindowAndBind(std::string, int, int, unsigned int)'

Thomas' Pangolin wrapper assumes Glut has been installed.  I needed to

    cmake -DFORCE_GLUT=ON ..
