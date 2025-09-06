# Edit files
#nano vignette_cuda.h
#nano vignette_cuda.cu
#nano v4l2-stream-test.cpp

# Compile CUDA vignette object
nvcc -O3 -std=c++20 -arch=sm_86 -c vignette_cuda.cu -o vignette_cuda.o

# Compile CUDA CRT filter
nvcc -O3 -std=c++20 -arch=sm_86 -c crt_filter_cuda.cu -o crt_filter_cuda.o

# Build client with C++20 and link CUDA runtime
g++ -O3 -std=c++20 v4l2-stream-test.cpp png_loader.cpp vignette_cuda.o crt_filter_cuda.o -o v4l2-stream-test -lpthread -lpopt -ljpeg -lpng -lcudart -fopenmp

# Run (example)
./v4l2-stream-test --upstream=192.168.168.175:1337 --listen=1337 --mjpeg --jpeg-quality=100 --no-lazy --encode-threads=6
#./v4l2-stream-test --upstream=192.168.168.175:1337 --listen=1337 --mjpeg --jpeg-quality=68 --no-lazy --encode-threads=10 --mjpeg --jpeg-quality=68 --no-lazy --encode-threads=10 --vig-center-x=0.5 --vig-center-y=0.5 --vig-inner=0.15 --vig-outer=0.45 --vig-strength=1.2 --vig-axis-x=1.0 --vig-axis-y=1.0 --vig-angle-deg=0
#./v4l2-stream-test --upstream=192.168.168.175:1337 --listen=1337 --mjpeg --jpeg-quality=100 --no-lazy --encode-threads=10 --mjpeg --jpeg-quality=68 --no-lazy --encode-threads=10 --vig-center-x=0.5 --vig-center-y=0.5 --vig-inner=0.073 --vig-outer=0.098 --vig-strength=1.85 --vig-axis-x=0.1957 --vig-axis-y=0.0471 --vig-angle-deg=0
