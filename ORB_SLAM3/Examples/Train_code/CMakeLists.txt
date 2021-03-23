cmake_minimum_required(VERSION 2.8)
PROJECT(INTEGRATION_CPP)

#SET(EXECUTABLE_OUTPUT_PATH ".")
find_package(Threads REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(CUDA 11.1 EXACT REQUIRED)

include_directories(/usr/local/zed/include)
include_directories(/usr/local/cuda/include)

link_directories(/usr/local/zed/lib)
link_directories(/usr/local/cuda/lib)

ADD_EXECUTABLE(main src/main.cpp)

#SET(ZED_LIBS /usr/local/zed /usr/local/cuda ${CUDA_CUDART_LIBRARY})
message("fdp" ${EXECUTABLE_OUTPUT_PATH} ${CMAKE_THREAD_LIBS_INIT})

TARGET_LINK_LIBRARIES(
    main
	/usr/local/cuda/lib64/libcudart.so
	/usr/local/zed/lib/libsl_zed.so
	/usr/lib/x86_64-linux-gnu/libopenblas.so
    /usr/lib/x86_64-linux-gnu/libusb-1.0.so
    /usr/lib/x86_64-linux-gnu/libcuda.so
    ${CMAKE_THREAD_LIBS_INIT}
)
