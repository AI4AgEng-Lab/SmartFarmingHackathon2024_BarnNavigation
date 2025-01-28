# Smart Farming Hackathon 2024 Barn Navigation

The Smart Farming Hackathon 2024 was presented by the Innovation Agency RLP. For 2 days we were part of a dynamic community of companies, start-ups and research institutions and experienced a lot. Our use case: Navigation of robots in a poultry house! We would like to share what we developed during the hackathon here :)

## Use-Case and Task

Simplified navigation for a robot:
- Concept development with subsequent implementation
- Autonomous control in a chicken barn
- without installations in the barn
- simple recurring paths

Development of an intelligent control system for an autonomous robot in an unknown environment in a poultry barn. The robot should autonomously and repeatedly follow a predefined route in the barn. The focus is on safe navigation in complex barns with enclosures, partition walls, manure removal systems and living animals.

More information: [Use_Case_Robotik.pdf](Use_Case_Robotik.pdf)

## Our Solution
First we developed a gazebo model for the poultry house. Then we used a TurtleBot3 as the basis for a robot. Instead of a classic LIDAR sensor, we then simulated four point lasers to meet the requirement for a simple sensor setup. Two of the lasers are attached to the front and two to the back of the robot. One of the two lasers in teh fron and in the back  looks orthogonally to the driving direction to the right, the other to the left. With the help of the two front lasers, the distance to the stables is regulated. All four lasers are also used to control the orientation of the robot in relation to the stables. In order not to give too much weight to individual incorrect measurements due to chickens flying past, a time series is measured for each laser and the mean value is used. The result can be sean in [video](barn_nav.mp4). For more information about the the robot control please look at [navigation_node.py](./workspace/src/barn_navigation/barn_navigation/navigation_node.py). 

In this case, we have chosen a solution without AI. However, for a more complex task with possibly more sensors, we consider an AI approach like fore instance rentforcemntlearning to be useful. In this case, the agend could first be trained in the simulation before it is then placed in a real poultry house. 

## Getting Started with the Code

### Docker
First select whether you work on Windows or Linux by commenting out the corresponding lines in the devcontainer file. 
Then build and open the devconatiner.
Open a new terminal in the conatiner an run.
```bash
   colcon build
```
In another terminal in the container tun:
```bash
   ros2 launch barn_navigation barn.launch.py 
```
to start the simulation.

## Authors

We were a team of four members and one supervisor.

Team Superviser: Jonas Boysen

Team Members: 
- Tim Lumpp
- Johanna Vaske
- Daniel Jiang
- Simon Mielke


## License

This project is licensed under the Apache License 2.0 - see the [License](LICENSE.md) file for details

## Acknowledgments

We would like to express our gratitude to the [Innovationsagentur Rheinland-Pfalz](https://innovationsagentur.rlp.de/) for organizing the hackathon. It was an incredible opportunity for us to participate and exchange innovative ideas.

We would also like to extend our thanks to [Big Dutchman](https://www.bigdutchman.com/de/startseite/) for providing the use case. We found it to be extremely interesting. Through this experience we learned a lot and the implementation of the use case greatly enhanced our understanding of the subject.
