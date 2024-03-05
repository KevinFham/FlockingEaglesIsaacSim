# FlockingEaglesNASAMinds

## Isaac Sim Documentation

#### Tutorials

https://docs.omniverse.nvidia.com/isaacsim/latest/index.html

https://docs.omniverse.nvidia.com/isaacsim/latest/isaac_gym_tutorials/tutorial_gym_isaac_gym.html


#### Python Docs

https://docs.omniverse.nvidia.com/py/isaacsim/index.html

Run the Isaac Sim local python interpreter (optionally with a predone `<script>`)

> `~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh <script>`

## Tips

Connect to a running simulation's python console through its port (default is `8223`)

> `telnet localhost <port>`

Shell scripts for conda and python are typically located at `~/.local/share/ov/pkg/isaac_sim-2023.1.1` ([How to set up](https://docs.omniverse.nvidia.com/isaacsim/latest/isaac_gym_tutorials/tutorial_gym_isaac_gym.html))


## Issues

"Detected a blocking function. This will cause hitches or hangs in the UI. Please switch to the async version" ([Source](https://forums.developer.nvidia.com/t/detected-a-blocking-function-this-will-cause-hitches-or-hangs-in-the-ui-please-switch-to-the-async-version/271191/12))

> Edit `~/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.core/omni/isaac/core/utils/nucleus.py` and change lines 178 to 198 with the function below

```py
def check_server(server: str, path: str, timeout: float = 10.0) -> bool:
    """Check a specific server for a path

    Args:
        server (str): Name of Nucleus server
        path (str): Path to search
        timeout (float): Literally does fucking nothing

    Returns:
        bool: True if folder is found
    """
    carb.log_info("Checking path: {}{}".format(server, path))
    # Increase hang detection timeout
    if "localhost" not in server:
        omni.client.set_hang_detection_time_ms(10000)
        result, _ = omni.client.stat("{}{}".format(server, path))
        if result == Result.OK:
            carb.log_info("Success: {}{}".format(server, path))
            return True
    carb.log_info("Failure: {}{} not accessible".format(server, path))
    return False
    
```
