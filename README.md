# Simulator Underwater

# Installation 

**WARING !!! :**

**It may be neccessary to install the library libprotobuf-c-dev, make sure you have it installed before installing the simulator**

**also:**
    
    Python-pip: Sudo apt-get install python-pip
    
    Python-scipi: sudo apt-get install python-scipy
    
    Casadi: sudo pip install casadi


Copy the files in your Workspace 

cd ~/name_of_workspace

    git clone https://github.com/castilloherrera/Simulator_Underwater.git

Then, build your workspace using

    cd ~/catkin_ws

    catkin_make install

or

    cd ~/catkin_ws

    catkin build

(in case you are using catkin_tools.)


# Start

Start an empty underwater environment using either

    roslaunch worlds ocean_waves.launch
