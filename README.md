# Kinematic Kitchen

Kinematic Kitchen is a robotics simulation of a high-throughput fast-food restaurant (digital twin). It automates the end-to-end burger assembly pipeline using **NVIDIA Isaac Sim** for physics and rendering and **ROS 2** for robot control.

The project is built on **Hexagonal Architecture (Ports and Adapters)**, which keeps the kitchen's business logic completely isolated from the simulation environment and robot hardware. The core domain never imports Isaac Sim or ROS 2 — all technology coupling is confined to adapters.

## Architecture

The codebase is split into these top-level packages:

```
hexagon/            — core domain: orders, recipes, kitchen state machine
driven_adapters/    — outbound adapters: persistence, robot notification, ID generation
driving_adapters/   — inbound adapters: CLI, ROS 2 nodes (future)
configuration/      — composition root: wires all adapters into the hexagon and runs the app
simulation/         — Isaac Sim scene, robot control helpers, custom ROS 2 message definitions
```

### Hexagon

The hexagon contains everything that is true regardless of technology:

- `hexagon/order.py` — the `Order` entity with its status machine (PENDING → IN_PROGRESS → COMPLETE)
- `hexagon/use_cases/` — one class per use case; each implements a driving port and depends only on driven ports
- `hexagon/driving_ports/` — abstract interfaces the outside world calls into the hexagon (e.g. `forSubmittingOrders`)
- `hexagon/driven_ports/` — abstract interfaces the hexagon calls out to infrastructure (e.g. `forPersistingOrders`, `forPreparingOrders`, `forGeneratingIds`)

No class inside `hexagon/` may import from `driven_adapters/`, `driving_adapters/`, or any external library. Non-deterministic concerns (ID generation, clocks) are driven ports injected at startup — never called directly.

### Adapters

Driven adapters implement driven ports and live under `driven_adapters/<port>/`:

| Adapter | Port | Technology |
|---|---|---|
| `forPersistingOrdersWithMemory` | `forPersistingOrders` | in-memory dict |
| `forPreparingOrdersWithIsaacSim` | `forPreparingOrders` | ROS 2 publisher → Isaac Sim |
| `forGeneratingIdsWithUuid` | `forGeneratingIds` | `uuid4` |

Driving adapters hold a driving port and trigger the hexagon from the outside. They live under `driving_adapters/<port>/`:

| Adapter | Port | Technology |
|---|---|---|
| `CliAdapter` | `forSubmittingOrders` | stdin / stdout |

### Configuration

`configuration/startup.py` is the composition root. It is the only place in the codebase that knows about all layers simultaneously. Its job is to instantiate adapters, inject them into the use cases, and start the driving adapter. Swapping a technology (e.g. replacing the in-memory store with a database) means changing one line here.

### Dependency flow

```
kinematic_kitchen  →  configuration
                            │
                ┌───────────┼───────────┐
                ▼           ▼           ▼
        driving_adapters  hexagon  driven_adapters
                            │
                    (ports only — no concrete imports)
```

Dependencies always point inward. The hexagon depends on nothing outside itself.

## Running the app

```bash
make submit                          # or: make submit ORDER="patty,bun,cheese"
```

This submits an order through the full stack and prints the generated order ID.

The composition root wires in `forPreparingOrdersWithIsaacSim`, which publishes
the order over ROS 2 as a `PrepareOrder` message, so the CLI needs `rclpy` and
the generated bindings on its path. `make submit` sources
[`simulation/ros_env.sh`](simulation/ros_env.sh) to provide both — see
[The ROS 2 environment](#the-ros-2-environment). Running it by hand is the same
two steps:

```bash
source simulation/ros_env.sh
"$KK_PYTHON" -m kinematic_kitchen submit "patty,bun,sauce"
```

## Running the simulation

The Isaac Sim scene (`simulation/kitchen_scene.py`) loads the kitchen from `assets/scene/KitchenSceneUr.usd`, tracks the scene's collision geometry as planning obstacles, and drives the UR10 arm and its suction cup with cuMotion, running a pick-and-place cycle whenever an order arrives over ROS 2.

### Prerequisites

- **NVIDIA Isaac Sim 6.0** installed locally (the commands below assume `~/isaacsim`; override with `ISAAC_SIM=/path/to/isaacsim`)
- **Docker**, used once to generate the message bindings
- A display: the scene opens the Isaac Sim GUI (`headless: False`)

A host ROS 2 installation is **not** required, and is not used even if present.

### The ROS 2 environment

Everything that speaks ROS 2 here — the scene, the CLI, the adapter tests — runs
on one distro: the **Jazzy** build that Isaac Sim bundles, on Isaac Sim's own
**Python 3.12**. [`simulation/ros_env.sh`](simulation/ros_env.sh) defines it and
exports `KK_PYTHON`, the interpreter those processes must use.

This is not a preference, it is forced by two ABI constraints that meet in the
middle:

- Isaac Sim 6.0 embeds **CPython 3.12**, and all of its extensions are `cp312`
  C extensions. The scene cannot run on another Python, and NVIDIA ships no
  build for one.
- `rclpy` also ships a compiled extension built for a single Python ABI, so a
  host ROS 2 on a different Python (Lyrical on Ubuntu 26.04 is 3.14) cannot be
  imported inside Isaac Sim at all.

Isaac Sim resolves this by bundling its own Jazzy `rclpy` for 3.12. Since Jazzy
is itself a Python 3.12 distro, using it on *both* sides keeps the project on a
single distro and lets the generated `PrepareOrder` bindings be built once and
shared by every process.

### Build the ROS 2 interfaces

`simulation/interfaces/` is an `ament_cmake` package holding the project's custom message definitions. Message types are generated by `rosidl` at build time, so this package is built with colcon rather than pip, and produces an overlay that has to be sourced by any process using those types:

```bash
make interfaces          # -> install_jazzy/
```

Re-run it after editing any `.msg` file — the bindings are generated code, so
until you rebuild, both the runtime and the editor see the old definition.

The build runs inside a `ros:jazzy-ros-base` container, so no ROS 2 install is
needed on the host. The container's Python is 3.12, the same minor version Isaac
Sim runs, which is what lets the result load directly into the scene.

For editor completion on `PrepareOrder`, `.vscode/settings.json` adds the generated package to `python.analysis.extraPaths`. It resolves only once the package has been built at least once.

### Start the scene

From the repository root:

```bash
./simulation/run_scene.sh
```

The launcher just sources `simulation/ros_env.sh` and starts the scene on
`KK_PYTHON`. It deliberately does not source a host ROS 2: putting a different
Python's `rclpy` on the path would shadow the bundled one and fail to import.

`make run` starts the scene, waits until a publisher can actually reach its
subscriber, and submits one order — all in one command. The first launch takes
noticeably longer (several minutes) while Isaac Sim compiles and caches shaders;
subsequent starts reach the main loop in about 20 seconds.

### Send an order

From a second terminal:

```bash
make submit ORDER="patty,bun,sauce"
```

Isaac Sim bundles `rclpy` but not the `ros2` command-line tools, so there is no
`ros2 topic pub` on this environment; `make submit` goes through the CLI, which
exercises the full stack anyway.

The sim logs `Starting pick and place for order <id>: <items>`, plans and executes a collision-free pick-and-place cycle, then logs `Order <id> completed`. Orders published while the arm is moving are queued and executed one after another.

If an order is never picked up, make sure `ROS_DOMAIN_ID` is the same (or unset)
in both terminals.

## Development

```bash
make install     # create .venv and install dev dependencies
make test        # run all tests
make lint        # check code style (ruff)
make typecheck   # run static type checker (mypy)
make docker-build  # build the Docker image
make docker-test   # run the test suite inside Docker
```

`make test` runs the whole suite on Isaac Sim's Python when that environment is
available, so the adapter tests exercise a real `rclpy` instead of skipping. It
falls back to the domain tests alone when it is not — which is what CI and the
Docker image do, since neither has Isaac Sim. The venv is only used for linting,
type checking and that fallback.

Tests are organised to mirror the source:

- `hexagon_tests/` — domain and use case tests (no infrastructure)
- `driven_adapters_tests/` — adapter tests exercised through the port interface
- `driving_adapters_tests/` — CLI and future driving adapter tests

CI runs automatically on every pull request to `develop` and `main`. A passing pipeline (lint + typecheck + tests + Docker build) is required before merging.
