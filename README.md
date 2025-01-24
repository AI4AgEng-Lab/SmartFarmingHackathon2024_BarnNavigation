# SmartFarmingHackathon2024_BarnNavigation

The Smart Farming Hackathon 2024 was presented by the Innovation Agency RLP. For 2 days we were part of a dynamic community of companies, start-ups and research institutions and experienced a lot. Our use case: Navigation of robots in a poultry house! We would like to share what we developed during the hackathon here :)

## Use-Case and Task

Simplified navigation for a robot:
- Concept development with subsequent implementation
- Autonomous control in a chicken barn
- without installations in the barn
- simple recurring paths

Development of an intelligent control system for an autonomous robot in an unknown environment in a poultry barn. The robot should autonomously and repeatedly follow a predefined route in the barn. The focus is on safe navigation in complex barns with enclosures, partition walls, manure removal systems and living animals

More information: [Use_Case_Robotik.pdf](Use_Case_Robotik.pdf)

## Getting Started

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

This project is licensed under the Apache License 2.0 - see the LICENSE.md file for details

## Acknowledgments

We would like to express our gratitude to the [Innovationsagentur Rheinland-Pfalz](https://innovationsagentur.rlp.de/) for organizing the hackathon. It was an incredible opportunity for us to participate and exchange innovative ideas.

We would also like to extend our thanks to [Big Dutchman](https://www.bigdutchman.com/de/startseite/) for providing the use case. We found it to be extremely interesting. Through this experience we learned a lot and the implementation of the use case greatly enhanced our understanding of the subject.
