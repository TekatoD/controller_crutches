## This file contains important information about building stuff for the robot
 
One should pass cmake arguments expicitly before starting generation. If crosscompile option is 
ON, paths to the 32bit versions of boost and OpenCV should be provided.
* DCROSSCOMPILE=ON/OFF - build for the simulator or for the robot                                          
* DOpenCV_DIR=/path/to/opencv/32/64 - path to the needed OpenCV version                               
* DBOOST_ROOT=/path/to/boost/32/64  - path to the needed boost version                                   

## Tips on building libraries
One should build libraries with special arguments for building binary file on 
the computer and for using libraries on the robot.
### This is the command that should be used to build OpenCV on the darwin itself.

````
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=./local -DCMAKE_C_FLAGS="-mfpmath=sse -flto -ffast-math" 
-DCMAKE_CXX_FLAGS="-mfpmath=sse -flto -ffast-math" -DBUILD_opencv_objdetect=ON -DWITH_ITT=OFF -DBUILD_opencv_python2=OFF 
-DBUILD_opencv_python3=OFF -DWITH_WEBP=OFF -DWITH_OPENEXR=OFF -DWITH_GSTREAMER=OFF -DWITH_GSTREAMER_0_10=OFF -DWITH_GPHOTO2=OFF 
-DWITH_MATLAB=OFF -DWITH_CUDA=OFF -DWITH_CUFT=OFF -DWITH_CUBLAS=OFF -DWITH_NVCUVID=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF 
-DENABLE_PRECOMPILED_HEADERS=OFF -DWITH_GTK=ON -DCPU_BASELINE_REQUIRE=SSE,SSE2 -DCPU_DISPATCH=SSE,SSE2 -DCPU_BASELINE_DISABLE=AVX,AVX2 ../

````
Also if Darwin running Ubuntu, one should place a a file, that contains path to this
OpenCV library to the */etc/ld.config.d/FILE_WITH_OPENCV_PATH.conf* and then run
````
sudo ldconfig
````

### This is the command that should be used to build 32 bit OpenCV on the developer's machine which.
````                                            #
 cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=./local -DCMAKE_C_FLAGS="-m32 -march=atom -mtune=atom -mfpmath=sse -flto
-ffast-math" -DCMAKE_CXX_FLAGS="-m32 -march=atom -mtune=atom  -mfpmath=sse -flto -ffast-math" -DBUILD_opencv_objdetect=OFF
-DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF -DWITH_WEBP=OFF -DWITH_OPENEXR=OFF  -DWITH_GSTREAMER=OFF -DWITH_GSTREAMER_0_10=OFF
-DWITH_GPHOTO2=OFF -DWITH_MATLAB=OFF -DWITH_CUDA=OFF -DWITH_CUFT=OFF -DWITH_CUBLAS=OFF -DWITH_NVCUVID=OFF -DWITH_GTK=OFF
-DCPU_BASELINE_REQUIRE=SSE,SSE2 -DCPU_BASELINE_DISABLE=AVX,AVX2 -DCPU_DISPATCH=SSE,SSE2 ../
````