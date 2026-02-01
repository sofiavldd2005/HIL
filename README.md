# HIL (Hardware-In-the-Loop) Simulation
A project dedicated to practicing Rust skills through the development of a Hardware-In-the-Loop (HIL) simulation. 
This **Hardware-in-the-Loop (HIL)** simulation framework aims to validate the flight software of the of a simple sounding rockets avionics avionics.
## Why Rust?
My background at RED has taught me that using the C programming language, whilist simpler, has tremendous costs in terms of memory safety and "debugabillity" of the software. I aim to:
- Eliminate **Race Conditions** during high-frequency sensor data injection.
- Provide **Memory Safety** without a garbage collector.
- Use **Async/Await (Embassy)** to model concurrent hardware behaviors with minimal overhead.
## Key Features :)
- **Deterministic Scheduling:** Ensuring simulation time steps matches the flight in real liofe.
- **Modular Drivers:** Easily switch between different sensors.
- **CLI Dashboard:** Maybe develop a dashboard for Real-time monitoring of the state machine.

## Documentation
The full technical documentation, including module definitions and protocol specifications, is automatically generated and hosted here:
**[View Full API Documentation](https://sofiavldd2005.github.io/HIL/HIL/index.html)**

