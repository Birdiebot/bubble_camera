<div align="center">
<img src=".github/cover.png"/>
<div>
    <a href=""><img src="https://img.shields.io/badge/Bubble-v1.0%20Developer%20Preview-blue" alt="version" /></a>
    <a href="https://hub.docker.com/repository/docker/birdiebot/bubble-aarch64v8"><img src="https://img.shields.io/docker/pulls/birdiebot/bubble-aarch64v8?logo=docker" alt="Docker Pulls"></a>
    <a href="https://www.gnu.org/licenses/agpl-3.0.en.html"><img src="https://img.shields.io/badge/license-GNU AGPL3.0-green" alt="license" /></a>
    <a href="https://birdiebot.github.io/bubble_documentation/"><img src="https://img.shields.io/badge/Documentation-completely-success" alt="documentation" /></a>
</div>
</div>

# Bubble camera
This package mainly maintains the SDK for the image input device in the [Bubble](https://github.com/Birdiebot/bubble) project. This package provides three image input methods: V4L, Daheng Industrial Camera, Hik-Robot Industrial Camera. You can use these drivers by switching between different branches. In the `main` branch, we maintain access to image data via V4L, which means you can quickly test you project on different platforms with USB cables or existing video.

As you can see due to time constraints, the different input methods are not derived via OOP, and the project currently maintains different SDK via branches. In the future, we may refactor this feature pack.

This project is a part of Bubble. For more information on the project, please visit [Bubble-RoboMaster visual software stack based on ROS2](https://github.com/Birdiebot/bubble).

# Documention
The documentation information you may need for Bubble core is listed below:
* [Bubble's Design Pattern](https://birdiebot.github.io/bubble_documentation/design/bubble%E8%AE%BE%E8%AE%A1%E6%A8%A1%E5%BC%8F.html)
* [API documention](https://birdiebot.github.io/bubble_documentation/API-documentation.html)

# About us
## About Bubble
Bubble is a ROS2-based open-source software stack made by the Birdiebot Team of the Mechanics and Science Innovation Base in Shanghai University of Engineering Science. Bubble provides a series of solution to the RMU related visual tasks.
## Author
Birdiebot is a school-level science and technology innovation team organized by the office of Academic Affairs in Shanghai University of Engineering Science. With the support of the Mechanics Innovation Base under Student Science and Technology Innovation Center, Birdiebot is committed to carry out science and technology innovation activities in robot competitions, robot ecology, engineering culture and other fields. To cultivate young, outstanding engineers for China’s intelligent manufacturing industry.

## Contributing
We welcome you to join us in the Bubble project to build the RoboMaster visual algorithm software ecosystem. We have some initial ideas about the future maintenance of the Bubble project, and you can participate in the maintenance of the Bubble project [here](https://birdiebot.github.io/bubble_documentation/resources/%E7%9B%B8%E5%85%B3%E9%A1%B9%E7%9B%AE.html).

## Contact
You can raise any questions on the Github Issues page or in the RoboMaster forum. For Visual Algorithm communication and Bubble project maintenance, you can contact us at algorithm@birdiebot.top. We also welcome other RoboMaster teams to communicate with us both online and offline. You can contact us at robomaster@birdiebot.top.

# License
For Bubble's references and other project used in Bubble can be found [here](https://birdiebot.github.io/bubble_documentation/resources/%E7%9B%B8%E5%85%B3%E9%A1%B9%E7%9B%AE.html).

The Bubble project is licensed under the [GNU AGPL3.0 License](https://www.gnu.org/licenses/agpl-3.0.en.html).

Copyright of Shanghai University of Engineering Science and Technology Student Science and Technology Center Mechanical Innovation Base Birdiebot Team. All rights reserved.
