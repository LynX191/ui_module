#!/bin/bash
BBlue='\033[1;34m'
BGreen='\033[1;32m'
BRed='\033[1;31m'
BYellow='\033[1;33m'
NC='\033[0m'

# OpenCV
opencvVer="4.5.5"
opencvLink="https://github.com/opencv/opencv.git"
contribLink="https://github.com/opencv/opencv_contrib.git"

#Ros
rosVer=""
isXavier=0

F_checkAndInstall () {
    versionCheck=$(dpkg -s $1 | grep Version)
    versionCheck=${versionCheck:9:-1}
    if [[ -n "$versionCheck" ]]; then
        echo -e "Already installed $1-$versionCheck"
    else
        sudo apt-get -y install $1
        versionCheck=$(dpkg -s $1 | grep Version)
        versionCheck=${versionCheck:9:-1}
        if [[ -z "$versionCheck" ]]; then
            echo -e "${BRed}Error: 'sudo apt-get -y install $1' failed!${NC}"
            exit 1
        fi
    fi
}

F_installPythonPackage () {
    readarray -d = -t strarr <<< "$1"
	A=${strarr[0]}
    versionCheck=$(pip3 list --format=columns | grep $A)
    if [[ -n "$versionCheck" ]]; then
        echo -e "Already installed $A"
    else
        python3 -m pip install $1
        versionCheck=$(pip3 list --format=columns | grep $A)
        if [[ -z "$versionCheck" ]]; then
            echo -e "${BRed}Install package '$1' failed!${NC}"
            exit 1
        fi
    fi
}

F_exportBashrc () {
    if ! grep -Fxq "$1" /home/$SUDO_USER/.bashrc; then
        echo $1 >> /home/$SUDO_USER/.bashrc
    fi
}

# Check Sudo bash
if [[ -z "$SUDO_USER" ]]; then
    echo -e "${BRed}Use 'sudo bash' before executing this script!${NC}"
    exit 1
fi

# Check ubuntu version
ubuntuVer=$(cat /etc/os-release | grep VERSION_ID | cut -d'"' -f2)
if [[ "$ubuntuVer" == "18.04" ]]; then
    rosVer="melodic"
else
    echo -e "${BRed}Error:  Does not support unbuntu version: '$ubuntuVer'!${NC}"
    exit 1
fi

# Check xavier or not
if [ -f /etc/nv_tegra_release ]; then
    isXavier=1
fi

sudo apt-get update
F_checkAndInstall "build-essential"
F_checkAndInstall "git"
F_checkAndInstall "checkinstall"
F_checkAndInstall "pkg-config"
F_checkAndInstall "python3-pip"

# OpenCV
sudo apt-get update
sudo apt-get -y install libhdf5-dev
sudo apt-get update
sudo apt-get -y install libhdf5-serial-dev
sudo apt-get update
sudo apt-get -y install libvtk6.3
opencvCheck=$(python3 -c 'import cv2; print(cv2.aruco)')
if [[ -z "$opencvCheck" ]]; then
    echo -e "Installing OpenCV $opencvVer"
    sleep 1
    pip3 uninstall opencv-python
	pip3 uninstall opencv-python-headless
	pip3 uninstall opencv-contrib-python
	pip3 uninstall opencv-contrib-python-headless
    opencvDir="/home/$SUDO_USER/Libraries/OpenCV_$opencvVer"
    mkdir -p $opencvDir && cd $opencvDir
    echo -e "OpenCV will be install in '$PWD'"
    sleep 1

    git clone $opencvLink -b $opencvVer
    git clone $contribLink -b $opencvVer
    cd opencv
    sudo apt-get update
    sudo apt purge -y libopencv-python libopencv-samples
    sudo apt-get -y install build-essential cmake git pkg-config libgtk-3-dev \
        libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
        libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
        gfortran openexr libatlas-base-dev python3-dev python3-numpy \
        libtbb2 libtbb-dev libdc1394-22-dev python3-pip

    mkdir -p build && cd build
    if [ -f "CMakeCache.txt" ]; then
        sudo rm CMakeCache.txt
    fi
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D_GLIBCXX_USE_CXX11_ABI=0 \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D BUILD_opencv_python2=OFF \
        -D PYTHON3_EXECUTABLE=$(which python3) \
        -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
        -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..

    sudo make -j$(($(nproc) - 1))
    sudo make install
    F_exportBashrc "# OpenCV"
    F_exportBashrc "export LD_LIBRARY_PATH=$opencvDir/opencv/build/lib:\$LD_LIBRARY_PATH"
    export LD_LIBRARY_PATH=$opencvDir/opencv/build/lib:$LD_LIBRARY_PATH
    opencvCheck=$(python3 -c 'import cv2; print(cv2.aruco)')
    if [[ -z "$opencvCheck" ]]; then
        echo -e "${BRed}Error: Install OpenCV failed!${NC}"
        exit 1
    fi
    echo -e "${BGreen}Install OpenCV-contrib successfully!${NC}"
    sleep 1
else
	echo -e "${BGreen}Installed OpenCV-contrib already!${NC}"
    sleep 1
fi

# Ros
rosCheck=$(dpkg -s ros-$rosVer-desktop-full | grep Version)
rosCheck=${rosCheck:9:-1}
if [[ -z "$rosCheck" ]]; then
    echo -e "${BGreen}Installing ros-$rosVer-desktop-full ...${NC}"
    sleep 1
    sudo apt-add-repository universe
    sudo apt-add-repository multiverse
    sudo apt-add-repository restricted
    F_checkAndInstall "curl"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-get update
    F_checkAndInstall "ros-$rosVer-desktop-full"
    F_exportBashrc "# Ros"
    F_exportBashrc "source /opt/ros/$rosVer/setup.bash"
    source /opt/ros/$rosVer/setup.bash
    echo -e "${BGreen}Install ros-$rosVer-desktop-full successfully!${NC}"
    sleep 1
else
    echo -e "${BGreen}Found ros-$rosVer-desktop-full-$rosCheck!${NC}"
    sleep 1
fi

# rosdep
sudo apt-get update
if [[ "$rosVer" == "melodic" ]]; then
    F_checkAndInstall "python-rosdep"
    F_checkAndInstall "python-rosinstall"
    F_checkAndInstall "python-rosinstall-generator"
    F_checkAndInstall "python-wstool"
    F_checkAndInstall "python-catkin-tools"
elif [[ "$rosVer" == "noetic" ]]; then
    F_checkAndInstall "python3-rosdep"
    F_checkAndInstall "python3-rosinstall"
    F_checkAndInstall "python3-rosinstall-generator"
    F_checkAndInstall "python3-wstool"
    F_checkAndInstall "python3-catkin-tools"
else
    echo -e "${BRed}Unrecognized Ros version: '$rosVer' ${NC}"
    exit 1
fi

# QTCreator
sudo apt-get update
F_checkAndInstall "qtcreator"
F_checkAndInstall "qt5-default"
F_checkAndInstall "qt5-doc"
F_checkAndInstall "qt5-doc-html"
F_checkAndInstall "qtbase5-doc-html"
F_checkAndInstall "qtbase5-examples"
F_checkAndInstall "qml-module-qtquick-controls2"
F_checkAndInstall "qml-module-qtmultimedia"
F_checkAndInstall "qml-module-qtquick-dialogs"
F_checkAndInstall "qml-module-qt-labs-platform"
F_checkAndInstall "qml-module-qt-labs-folderlistmodel"
F_checkAndInstall "qml-module-qt-labs-settings"
F_checkAndInstall "qtdeclarative5-dev"
F_checkAndInstall "qml-module-qtquick-virtualkeyboard"
F_checkAndInstall "qtvirtualkeyboard-plugin"


# BIXOLON printer
F_checkAndInstall "cups"
F_checkAndInstall "libcups2-dev"

# depthai-ros
sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
sudo rosdep fix-permissions
rosdep update
sudo apt-get update

depthaiFile="/usr/local/include/depthai/depthai.hpp"
if ! [ -f $depthaiFile ]; then
	if [[ $isXavier == 1 ]]; then
		F_exportBashrc "# depthai-core"
		F_exportBashrc "export OPENBLAS_CORETYPE=ARMV8"
		export OPENBLAS_CORETYPE=ARMV8
	fi
	rm -rf ~/.hunter
	set -e
	echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger
	cd /tmp
    if [ -d depthai-core ]; then
        rm -rf depthai-core
    fi
	git clone --recursive https://github.com/luxonis/depthai-core.git --branch main
    opencv4Dir="/usr/local/include/opencv4/"
    opencv2Dir="/usr/local/include/opencv2/"
    xavierOpencv4Dir="/usr/include/opencv4"
    xavierOpencv2Dir="/usr/include/opencv2"
    opencvDir=""
    if [ -d $xavierOpencv4Dir ]; then
        opencvDir=$xavierOpencv4Dir
    elif [ -d $opencv4Dir ]; then
        opencvDir=$opencv4Dir
    elif [ -d $xavierOpencv2Dir ]; then
        opencvDir=$xavierOpencv2Dir
    elif [ -d $opencv2Dir ]; then
        opencvDir=$opencv2Dir
    else
        echo -e "${BRed}Could not found Opencv for depthai!${NC}"
        exit 0
    fi
    cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local -DOpenCV_DIR=$opencvDir
	cmake --build depthai-core/build --target install -- -j$(($(nproc) - 1))
	if ! [ -f $depthaiFile ]; then
		echo -e "${BRed}Error: Install depthai-core failed!${NC}"
        exit 0
	fi
	cd /tmp
	rm -r depthai-core
    echo -e "${BGreen}Installed depthai-core!${NC}"
    sleep 1
else
	echo -e "${BGreen}Found depthai-core${NC}"
    sleep 1
fi

# Fix missed package
sudo apt-get update
sudo apt-get -y install libxmlrpcpp-dev --fix-missing
sudo apt-get -y install librosconsole-dev --fix-missing
sudo apt-get -y install libssh-dev --fix-missing
sudo apt-get -y install sshpass --fix-missing
sudo apt-get -y install qml-module-qtquick-extras --fix-missing
sudo apt-get -y install expect --fix-missing
sudo apt-get -y install oping --fix-missing

# git clone https://git.libssh.org/projects/libssh.git
cd libssh/
mkdir build
cd build/
cmake ..
make
sudo make install


# Uninstall local setuptools
pythonPathCheck=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())")
readarray -d / -t strarr <<< "$pythonPathCheck"
if [[ "${strarr[2]}" == "local" ]]; then
    pip3 uninstall -y setuptools
fi

# Change cv_brige
opencvDir=""
opencvDir1="/usr/include/opencv4"
opencvDir2="/usr/local/include/opencv4"
opencvDir3="/usr/include/opencv2"
opencvDir4="/usr/local/include/opencv2"
if [ -d $opencvDir1 ]; then
    opencvDir="\/usr\/include\/opencv4"
elif [ -d $opencvDir2 ]; then
	opencvDir="\/usr\/local\/include\/opencv4"
elif [ -d $opencvDir3 ]; then
    opencvDir="\/usr\/include\/opencv2"
elif [ -d $opencvDir4 ]; then
	opencvDir="\/usr\/local\/include\/opencv2"
fi
if [[ "$opencvDir" != "" ]]; then
	cvBrigeFile="/opt/ros/$rosVer/share/cv_bridge/cmake/cv_bridgeConfig.cmake"
	search="_include_dirs \\\"include\;\/usr\/include\;\/usr\/include\/opencv\\\""
	replace="_include_dirs \\\"include\;\/usr\/include\;$opencvDir\\\""
    echo "s/$search/$replace"
	sed -i "s/$search/$replace/g" $cvBrigeFile
fi

