# ros_compatibility
ros_compatibility is a wrapper for both ROS1 and ROS2 that compiles the correct version from CMake. The support of ros_compatibility extends both to some basic constructs of ROS (e.g., the time and duration primitives present in ROS1 that have been removed) and to some specific messages. 
## Wrapped features
ROS version independence is provided for the following basic constructs:

* `Node`
* `Subscriber and Publisher`
* `Executor`
* `Time`
* `Action`
* `tf2`
* `bag`

<details>
<summary>supported msgs</summary>

* `ackermann_msgs`
* `actuation_msgs`
* `geometry_msgs`
* `nav_msgs`
* `sensor_msgs`
* `std_msgs`
* `std_srvs`
* `visualization_msgs`
* `wiimote_msgs`

</details>

<details>
<summary>ROS1 specific msgs</summary>

* `actionlib_msgs`
* `gps_common`

</details>

<details>
<summary>ROS2 specific msgs</summary>

* `action_msgs`
* `gps_msgs`
* `nav2_msgs`

</details>


## Authors
* **Michele Guzzinati** - [mguzzina](https://github.com/mguzzina)
* **Federico Gavioli** - [fgavioli](https://github.com/fgavioli)
* **Andrea Bernardi** - [andreabernard](https://github.com/andreabernard)
* **Antonio Russo** - [russoanto](https://github.com/russoanto)

## Project Managers
* **Paolo Burgio** - [pburgio](https://github.com/pburgio)

## License
**Apache 2.0** - [License](https://opensource.org/licenses/Apache-2.0)

## AD Project
This repository is part of the autonomous driving project of the University of Modena and Reggio Emilia, [read more](https://hipert.github.io/ad_site/).