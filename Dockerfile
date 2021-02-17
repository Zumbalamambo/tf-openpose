FROM ros:melodic as melodic-cuda10

ENV PATH /usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# cuda
RUN apt-get update && apt-get install wget && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin && \
    mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget http://developer.download.nvidia.com/compute/cuda/10.2/Prod/local_installers/cuda-repo-ubuntu1804-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb && \
    sudo dpkg -i cuda-repo-ubuntu1804-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb && \
    sudo apt-key add /var/cuda-repo-10-2-local-10.2.89-440.33.01/7fa2af80.pub && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install cuda cuda-drivers && \
    rm cuda-repo-ubuntu1804-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb

# cudnn
RUN wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb && \
    apt-get install ./nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb && \
    apt-get update && \
    apt-get install libcudnn7 libcudnn7-dev && \
    rm nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

FROM melodic-cuda10

ENV PYENV_ROOT $HOME/.pyenv
ENV PATH $PYENV_ROOT/bin:${PATH}

RUN apt-get install -y git build-essential libffi-dev libssl-dev zlib1g-dev liblzma-dev libbz2-dev libreadline-dev libsqlite3-dev python3-pip && \
    git clone https://github.com/pyenv/pyenv.git $PYENV_ROOT && \
    echo 'eval "$(pyenv init -)"' >> ~/.bashrc && \
    eval "$(pyenv init -)" && \
    pip3 install pipenv

RUN apt-get update && \
    apt-get install -y gcc-7 python-catkin-tools python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge && \
    pip3 install rospkg empy

RUN mkdir ros &&\
    cd ros &&\
    catkin init &&\
    catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so &&\
    git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv &&\
    cd src/vision_opencv/ &&\
    git checkout melodic && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash;catkin build" && \
    echo "source /ros/devel/setup.bash" >> /root/.bashrc

RUN cd /usr/local/cuda/lib64/ && \
    cp libcudart.so libcudart.so.10.0 && \
    cp libcufft.so libcufft.so.10.0 && \
    cp libcufft.so libcufft.so.10.0 && \
    cp libcurand.so libcurand.so.10.0 && \
    cp libcusolver.so libcusolver.10.0 && \
    cp libcusparse.so libcusparse.10.0 && \
    cp libcusolver.so libcusolver.so.10.0 && \
    cp libcusparse.so libcusparse.so.10.0 && \
    cd /usr/lib/x86_64-linux-gnu/ && \
    cp libcublas.so libcublas.so.10.0

COPY . /ros/src/tf-pose-estimation

RUN cd /ros/src/tf-pose-estimation && \
    pipenv install --python 3.6 && \
    apt-get install -y swig && \
    pipenv run bash -c "cd src/tfpose_ros/pafprocess && swig -python -c++ pafprocess.i && python3 setup.py build_ext --inplace"

RUN cd /ros && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash;catkin build"

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
