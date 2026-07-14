# Kinematic Kitchen

Kinematic Kitchen is a robotics simulation of a high-throughput fast-food restaurant (digital twin). It automates the end-to-end burger assembly pipeline using **NVIDIA Isaac Sim** for physics and rendering and **ROS 2** for robot control.

The project is built on **Hexagonal Architecture (Ports and Adapters)**, which keeps the kitchen's business logic completely isolated from the simulation environment and robot hardware. The core domain never imports Isaac Sim or ROS 2 — all technology coupling is confined to adapters.

## Architecture

The codebase is split into four packages at the root level:

```
hexagon/            — core domain: orders, recipes, kitchen state machine
driven_adapters/    — outbound adapters: persistence, robot notification, ID generation
driving_adapters/   — inbound adapters: CLI, ROS 2 nodes (future)
configuration/      — composition root: wires all adapters into the hexagon and runs the app
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
| `forPreparingOrdersWithIsaacSim` | `forPreparingOrders` | Isaac Sim stub |
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
python -m kinematic_kitchen submit "patty,bun,sauce"
```

This submits an order through the full stack and prints the generated order ID alongside a confirmation that the robot was notified.

## Running the simulation

The Isaac Sim scene (`assets/simulation/kitchen_scene.py`) loads the kitchen, tracks the scene's collision geometry as planning obstacles, and moves the Franka arm with cuMotion whenever an order arrives over ROS 2.

### Prerequisites

- **NVIDIA Isaac Sim 6.0** installed locally (the commands below assume `~/isaacsim`; adjust to your install path)
- **ROS 2 Jazzy** at `/opt/ros/jazzy` (its Python must match Isaac Sim's — both use Python 3.12)
- A display: the scene opens the Isaac Sim GUI (`headless: False`)

### Start the scene

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=$(pwd):$PYTHONPATH ~/isaacsim/python.sh assets/simulation/kitchen_scene.py
```

Sourcing ROS 2 first puts `rclpy` on the path for Isaac Sim's bundled Python; **append** to `PYTHONPATH` rather than overwrite it, or the ROS 2 packages are lost again. The first launch downloads the Franka asset from NVIDIA's servers, so it needs network access and takes a little longer.

### Send an order

From a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /kinematic_kitchen/prepare_order std_msgs/String "{data: 'order-1'}"
```

The sim logs `Starting motion for order <id>` and the arm plans and executes a collision-free motion to the reach pose. Orders published while the arm is moving are queued and executed one after another.

If the topic does not appear in `ros2 topic list`, make sure `ROS_DOMAIN_ID` is the same (or unset) in both terminals.

## Development

```bash
make install     # create .venv and install dev dependencies
make test        # run all tests
make lint        # check code style (ruff)
make typecheck   # run static type checker (mypy)
make docker-build  # build the Docker image
make docker-test   # run the test suite inside Docker
```

Tests are organised to mirror the source:

- `hexagon_tests/` — domain and use case tests (no infrastructure)
- `driven_adapters_tests/` — adapter tests exercised through the port interface
- `driving_adapters_tests/` — CLI and future driving adapter tests

CI runs automatically on every pull request to `develop` and `main`. A passing pipeline (lint + typecheck + tests + Docker build) is required before merging.
