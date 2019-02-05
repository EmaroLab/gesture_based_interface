# Activation


# Requirements:
* Linux Ubuntu 18.04
* Ros Melodic
* Kinect 360
* Beacon

# Setup:

It is requested to install [freenect]

# The node check if the beacons reveal operator's presence, and if the operator is in front of the Baxter.
* If both the conditions are satisfied, it sends an activate signal to the Baxter.
* Additionally, it checks continuously the security distance between the operator and the baxter.
```
