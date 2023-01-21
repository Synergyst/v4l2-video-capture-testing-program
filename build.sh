#/bin/bash

export PATH=/usr/local/cuda-10.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:/usr/local/lib:/usr/local/cuda-10.2/targets/aarch64-linux/lib:$LD_LIBRARY_PATH

#(git clone git://git.ideasonboard.org/uvc-gadget.git && cd uvc-gadget && make && make install)
#if [[ $? -eq 0 ]]; then
#  echo "Successfully built (uvc-gadget)"
#else
#  echo "Build failed (uvc-gadget), perhaps we already built it?"
#fi

#g++ -shared -o ~/projects/v4l2-capture-test/bin/ARM/Debug/libv4l2edid.so -fPIC -O3 -std=gnu++20 -lv4l2 v4l2edid.cpp
#if [[ $? -eq 0 ]]; then
#  echo "Successfully built: libv4l2edid.so"
#  #echo "We need to copy the libv4l2edid.so library file to /usr/local/lib/ and then run ldconfig"
#  sudo cp ~/projects/v4l2-capture-test/bin/ARM/Debug/libv4l2edid.so /usr/local/lib/ && sudo ldconfig
#else
#  echo "Build failed (libv4l2edid.so)"
#  exit 1
#fi

#g++ -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test -O3 -std=gnu++20 -fopenmp v4l2-capture-test.cpp -ldl -lv4l2 -lpthread
/usr/local/cuda-10.2/bin/nvcc -I/usr/local/cuda-10.2/include -I/usr/local/cuda-10.2/samples/common/inc -L/usr/local/cuda-10.2/lib64 -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test -O3 v4l2-capture.cu -ldl -lv4l2 -lpthread -lcudart
#apt-get install cuda-toolkit-10-2 cuda-tools-10-2 cuda-compiler-10-2 cuda-samples-10-2 cuda-libraries-10-2 cuda-driver-dev-10-2 libcudnn8-dev libcudnn8 cuda-*dev* mlocate dos2unix
if [[ $? -eq 0 ]]; then
  echo "Successfully built: v4l2-capture-test"
else
  echo "Build failed (v4l2-capture-test)"
  exit 1
fi

#~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test /dev/video0 640 360 | ffplay -hide_banner -loglevel error -f rawvideo -video_size 640x360 -pixel_format gray -i pipe:0

#rustc -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-video-capture-testing-program-porting v4l2-video-capture-testing-program-porting.rs
#if [[ $? -eq 0 ]]; then
#  echo "Successfully built: v4l2-video-capture-testing-program-porting"
#else
#  echo "Build failed (v4l2-video-capture-testing-program-porting)"
#  exit 1
#fi