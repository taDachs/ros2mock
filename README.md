# ros2mock

Tool for mocking ROS 2 services and actions.

## Usage

**Mock a service**

```bash
ros2 mock service foobar std_srvs/srv/Trigger --timeout 10.0
```

Creates a service server that responds with default response and has a delay of 10s.

You can also specify a custom response:

```bash
ros2 mock service foobar std_srvs/srv/Trigger "{success: True, message: 'Foobar'}"
```

**Mock an action**

```bash
ros2 mock action /foobar example_interfaces/action/Fibonacci
```

You can also specify feedback messages and the result: 

```bash
ros2 mock action /foobar example_interfaces/action/Fibonacci "sequence: [0, 1]" --feedback "sequence: [0]"
```

This action sends a single feedback message. You can specify `--feedback` multiple times to define
multiple feedback messages that all get send in sequence.
