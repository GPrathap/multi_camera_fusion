FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu14.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates apt-transport-https gnupg-curl && \
    rm -rf /var/lib/apt/lists/* && \
    NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \
    NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/7fa2af80.pub && \
    apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +2 > cudasign.pub && \
    echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64 /" > /etc/apt/sources.list.d/cuda.list

ENV CUDA_VERSION 8.0.61

ENV CUDA_PKG_VERSION 8-0=$CUDA_VERSION-1
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-nvrtc-$CUDA_PKG_VERSION \
        cuda-nvgraph-$CUDA_PKG_VERSION \
        cuda-cusolver-$CUDA_PKG_VERSION \
        cuda-cublas-8-0=8.0.61.2-1 \
        cuda-cufft-$CUDA_PKG_VERSION \
        cuda-curand-$CUDA_PKG_VERSION \
        cuda-cusparse-$CUDA_PKG_VERSION \
        cuda-npp-$CUDA_PKG_VERSION \
        cuda-cudart-$CUDA_PKG_VERSION && \
    ln -s cuda-8.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# nvidia-docker 1.0
LABEL com.nvidia.volumes.needed="nvidia_driver"
LABEL com.nvidia.cuda.version="${CUDA_VERSION}"

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=8.0"

RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-core-$CUDA_PKG_VERSION \
        cuda-misc-headers-$CUDA_PKG_VERSION \
        cuda-command-line-tools-$CUDA_PKG_VERSION \
        cuda-nvrtc-dev-$CUDA_PKG_VERSION \
        cuda-nvml-dev-$CUDA_PKG_VERSION \
        cuda-nvgraph-dev-$CUDA_PKG_VERSION \
        cuda-cusolver-dev-$CUDA_PKG_VERSION \
        cuda-cublas-dev-8-0=8.0.61.2-1 \
        cuda-cufft-dev-$CUDA_PKG_VERSION \
        cuda-curand-dev-$CUDA_PKG_VERSION \
        cuda-cusparse-dev-$CUDA_PKG_VERSION \
        cuda-npp-dev-$CUDA_PKG_VERSION \
        cuda-cudart-dev-$CUDA_PKG_VERSION \
        cuda-driver-dev-$CUDA_PKG_VERSION && \
    rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs


RUN echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1404/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDNN_VERSION 7.1.4.18

LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
            libcudnn7=$CUDNN_VERSION-1+cuda8.0 \
            libcudnn7-dev=$CUDNN_VERSION-1+cuda8.0 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends mesa-utils

# Run installers.
COPY installers /tmp/installers
RUN bash /tmp/installers/pre_install.sh
RUN bash /tmp/installers/install_adv_plat.sh
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_bazel_packages.sh
RUN bash /tmp/installers/install_conda.sh
RUN bash /tmp/installers/install_gflags_glog.sh
RUN bash /tmp/installers/install_glew.sh
RUN bash /tmp/installers/install_glusterfs.sh
RUN bash /tmp/installers/install_gpu_caffe.sh
RUN bash /tmp/installers/install_ipopt.sh
RUN bash /tmp/installers/install_libjsonrpc-cpp.sh
RUN bash /tmp/installers/install_nlopt.sh
RUN bash /tmp/installers/install_node.sh
RUN bash /tmp/installers/install_ota.sh
RUN bash /tmp/installers/install_pcl.sh
RUN bash /tmp/installers/install_protobuf.sh
RUN bash /tmp/installers/install_python_modules.sh
RUN bash /tmp/installers/install_qp_oases.sh
RUN bash /tmp/installers/install_ros.sh
RUN bash /tmp/installers/install_snowboy.sh
RUN bash /tmp/installers/install_supervisor.sh
RUN bash /tmp/installers/install_undistort.sh
RUN bash /tmp/installers/install_user.sh
RUN bash /tmp/installers/install_yarn.sh
RUN bash /tmp/installers/install_glfw3_2.sh
RUN bash /tmp/installers/post_install.sh

RUN apt-get install -y \
   bc \
   cppcheck \
   debconf-utils \
   doxygen \
   graphviz \
   gdb \
   git \
   subversion \
   google-perftools \
   lcov \
   libblas-dev \
   libboost-all-dev \
   libcurl4-openssl-dev \
   libfreetype6-dev \
   liblapack-dev \
   libpcap-dev \
   locate \
   lsof \
   realpath \
   shellcheck \
   vim \
   v4l-utils \
   nfs-common \
   zip

RUN apt-get remove libgflags-dev -y

RUN deluser apollo

WORKDIR /apollo

