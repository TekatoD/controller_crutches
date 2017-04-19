set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR atom)

set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32 -march=atom -mtune=atom -mfpmath=sse -flto -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32 -march=atom -mtune=atom  -mfpmath=sse -flto -ffast-math")
