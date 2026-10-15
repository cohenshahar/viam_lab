# Glossary

| Term | What it actually is |
| --- | --- |
| viam-server | The program on the robot's computer that runs everything |
| Machine | One robot (or device) in the Viam app, with its own config |
| Component | A piece of hardware your machine controls: an arm, a gripper, a camera |
| Service | A capability that runs on top of components: motion planning, vision, data capture |
| Resource | The word for either one (component or service); everything configured on a machine is a resource |
| Module | The code that implements one or more resources. viam-server downloads modules from the registry and runs them |
| Registry | Viam's shared store of modules, ML models and fragments |
| Fragment | A reusable piece of config |
| Frame system | The tree of coordinate frames: where each part sits relative to the others |
| SDK (Software Development Kit) | The library you code with (Python here) |
| OTA (Over-The-Air) | Pushing updates to robots over the network |
