# pilot_net_controller

ROS2 inference package for PilotNet (end-to-end steering). Runs on NumPy only —
no PyTorch dependency — loading weights exported by
[`pilot_net_training`](../pilot_net_training) from `weights/pilotnet_weights.npy`.

```
pilot_net_controller/
├── config/pilot_net_node.param.yaml
├── launch/pilot_net.launch.xml
├── weights/pilotnet_weights.npy   # deployed by pilot_net_training/convert_weight.py
└── pilot_net_controller/
    ├── pilot_net_controller_node.py   # rclpy.Node: image in, control command out
    ├── pilot_net_controller_core.py   # preprocessing + inference + postprocessing
    └── model/
        ├── pilotnet.py                # PilotNetNp forward pass
        └── numpy/
            └── layers.py               # conv2d / linear / relu / tanh primitives
```
