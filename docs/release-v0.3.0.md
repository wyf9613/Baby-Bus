# AI4R Policy v0.3.0 release preparation

This source release groups the student configuration/documentation changes
already integrated on dev with eight new Traxxas output-limit parameters.
It includes optional lidar mounting configuration, Traxxas steering/input/
wheel settings, and clearer observation/action comments. Policy algorithms are
unchanged by these changes; the exact IDL dependency remains in
`ci/dependencies.repos`.

Use DREAM 0.8.0 and Traxxas firmware/interface 0.5.0 for the active output-limit
keys in `config/traxxas_vehicle_interface.yaml`. Older DREAM compositions do
not delegate these keys and reject them. Omit the new keys only when deliberately
using an older compatible system. Changes take effect when DREAM restarts the
vehicle interface, which reapplies and verifies the complete set while starting
Disarmed. This policy never enables the vehicle.

The complete synthetic ROS gate passed 65 tests on the supplied Jetson. No
powered vehicle behavior or full composed robot run is claimed. Publication
requires recorded human review, the frozen candidate gate, identical-tree
non-squashed promotion, an annotated v0.3.0 tag and verified final-tag artifacts.
