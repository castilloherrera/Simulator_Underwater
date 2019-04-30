#***************************************************    
# * Title: UUV Simulator   
# * Author: The UUV Simulator Authors  
# * Date: 2016      
# * Availability: https://uuvsimulator.github.io/
#***************************************************

# This represents a position in free space with uncertainty.

geometry_msgs/Point pos

# Row-major representation of the 3x3 covariance matrix
float64[9] covariance
