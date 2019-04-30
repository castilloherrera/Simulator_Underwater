#***************************************************    
# * Title: UUV Simulator   
# * Author: The UUV Simulator Authors  
# * Date: 2016      
# * Availability: https://uuvsimulator.github.io/
#***************************************************

Header header
# Sensor link position
geometry_msgs/Vector3 position
# Latitude in degrees (WGS84 coordinates)
float64 latitude
# Longiture in degrees (WGS84 coordinates)
float64 longitude
# Depth in meters (WGS84 coordinates)
float64 depth
# Particle concentration
float64 concentration
# Status, false if no plume particles are received
bool is_measuring
