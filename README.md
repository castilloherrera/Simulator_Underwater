# Simulator Underwater

# Installation 

**WARING !!! :**

**It may be neccessary to install the library libprotobuf-c-dev, make sure you have it installed before installing the simulator**

**also**
    
    Python-pip: sudo apt-get install python-pip
    
    Python-scipi: sudo apt-get install python-scipy
    
    Casadi: sudo pip install casadi

After that,

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
    
    
**GAZEBO ERROR (Solution)**
  
**Error in RESET request for accessing api.ignition.org**

In terminal
  
    sudo gedit ~/.ignition/fuel/config.yaml
    
In url, update the server from https://api.ignitionfuel.org to https://api.ignitionrobotics.org
