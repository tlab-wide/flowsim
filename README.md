Here's the updated README.md based on the provided Flowsim paper and the existing README.md file:

# Flowsim

Flowsim is a novel simulator specifically designed to evaluate the behavior and data flow of connected autonomous vehicles (CAVs) in large-scale environments. It offers a modular and extensible architecture that enables the analysis of CAV behaviors in city-scale scenarios.

Flowsim is written in <strong>pure Python</strong> within approximately <strong>1,500</strong> lines of code, making it highly readable, understandable, and easily modifiable.
The purpose is to enable researchers to fast-prototype, simulate, and test CAV algorithms and functions. 
By implementing the desired modules and defining flow between them, users can create custom vehicle types very easily.

The key features of Flowsim are:

* <strong>City-scaled</strong>: Flowsim provides a city-scale experimental environment for evaluating CAV behaviors in realistic traffic scenarios.
* <strong>Extensible</strong>: Flowsim offers a modular and extensible architecture that allows easy integration of new simulation components and customization of vehicle types and behaviors.
* <strong>Flexible</strong>: Flowsim enables researchers to define custom data flows and implement desired modules to create custom vehicle types and analyze their interactions.
* <strong>High-performance</strong>: Flowsim achieves high-performance simulation capabilities, enabling efficient evaluation of large-scale scenarios with thousands of vehicles.
* <strong>Reproducible</strong>: Flowsim ensures reproducibility of experiments by providing a well-documented and accessible codebase, fostering collaboration and knowledge sharing within the research community.

## Major Components
![Flowsim structure][docs/images/overview.pdf]

Flowsim consists of three major components: <strong>Environment</strong>,  <strong>Sandbox</strong>, and <strong>Vehicle</strong>s.

* <strong>Environment</strong>: The environment component includes various simulators such as PositionManager, NetworkSimulator, PerceptionSimulator, and MatchSimulator. These simulators provide the necessary information and functionality for evaluating CAV behaviors.
* <strong>Sandbox</strong>: The sandbox component ensures the legitimacy of vehicle implementations by explicitly controlling the information accessible to agents and the operations they can perform.
* <strong>Vehicles</strong>: Flowsim supports different types of vehicles, including general-purpose vehicles, V2X-connected vehicles, Proof-of-Traffic (PoT) vehicles, and attacker vehicles with tampered OBU implementations. The data flow among vehicle modules is represented by a Directed Acyclic Graph (DAG).

## Citation
If you are using the Flowsim framework for your research or development, please cite the following paper:

Y. Tao, E. Javanmardi, J. Nakazato, M. Tsukada and H. Esaki, "Flowsim: A Modular Simulation Platform for Microscopic Behavior Analysis of City-Scale Connected Autonomous Vehicles," 2023 IEEE 26th International Conference on Intelligent Transportation Systems (ITSC), Bilbao, Spain, 2023, pp. 2186-2193, doi: 10.1109/ITSC57777.2023.10421900.

BibTeX entry:
```bibtex
@INPROCEEDINGS{10421900,
  author={Tao, Ye and Javanmardi, Ehsan and Nakazato, Jin and Tsukada, Manabu and Esaki, Hiroshi},
  booktitle={2023 IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)},
  title={Flowsim: A Modular Simulation Platform for Microscopic Behavior Analysis of City-Scale Connected Autonomous Vehicles},
  year={2023},
  volume={},
  number={},
  pages={2186-2193},
  keywords={Codes;Protocols;Transportation;Behavioral sciences;Computer security;Autonomous vehicles;Software development management},
  doi={10.1109/ITSC57777.2023.10421900}
}
```

## Get Started!

### User Guide
TBD

### Developer Guide

![Defining vehicle types][docs/images/vehicle_types.pdf]


### Contribution Guidelines
We welcome your contributions to Flowsim.
- Please report bugs and suggest improvements by submitting issues on the GitHub repository.
- Submit your contributions using [pull requests](https://github.com/tlab-wide/Flowsim/pulls).
- Ensure your code follows the established coding style and conventions used in the Flowsim codebase.
- Provide clear and concise descriptions of your changes, along with any necessary documentation updates.
