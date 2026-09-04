# Recipes source ROS 2's setup.bash, which uses bash-only syntax, so run them
# under bash rather than the default /bin/sh (dash).
SHELL := /bin/bash

VENV := .venv
PYTHON := $(VENV)/bin/python
PIP := $(VENV)/bin/pip

# Overridable locations for the live `run` target (Isaac Sim + ROS 2).
ISAAC_SIM ?= $(HOME)/isaacsim
ROS_SETUP ?= /opt/ros/lyrical/setup.bash
SCENE := simulation/run_scene.sh
TOPIC := /kinematic_kitchen/prepare_order
ORDER ?= patty,bun,sauce

# colcon overlay holding the generated PrepareOrder bindings. Every process
# that touches the topic -- the scene, the CLI, the adapter tests -- needs it
# sourced on top of ROS 2, or the message type cannot be imported.
INTERFACES := simulation/interfaces
OVERLAY_SETUP := install/setup.bash

# The scene runs on Isaac Sim's bundled Python 3.12 and its bundled Jazzy ROS 2,
# which is a different distro and ABI from the system ROS 2. The same .msg
# therefore has to be built twice: once for the system distro (the `interfaces`
# target above, used by the CLI) and once for Jazzy/3.12 (this one, used by the
# scene). The Jazzy build runs in a container so no second ROS 2 install is
# needed on the host; the two builds interoperate over DDS.
SIM_MSGS_IMAGE ?= ros:jazzy-ros-base
SIM_INSTALL := install_jazzy

.PHONY: install interfaces interfaces-sim lint lint-domain typecheck typecheck-domain test test-domain run docker-build docker-test

# --system-site-packages lets the venv see apt-installed packages (numpy, yaml)
# that ROS 2's rclpy depends on, so the Isaac Sim adapter tests can run locally
# when ROS 2 is sourced. CI never sources ROS 2 and runs domain tests only.
$(VENV):
	python3 -m venv --system-site-packages $(VENV)

install: $(VENV)
	$(PIP) install -e ".[dev]" -q

# Regenerate the custom message bindings. Re-run after editing any .msg file.
interfaces:
	@if [ ! -f "$(ROS_SETUP)" ]; then \
		echo "ROS 2 setup not found at $(ROS_SETUP). Set ROS_SETUP=/path/to/setup.bash" >&2; exit 1; fi
	@set -e; . "$(ROS_SETUP)"; colcon build --base-paths $(INTERFACES)

interfaces-sim:
	@if ! docker info >/dev/null 2>&1; then \
		echo "Docker is required to build the simulator's messages (image: $(SIM_MSGS_IMAGE))." >&2; exit 1; fi
	docker run --rm -u $$(id -u):$$(id -g) -v "$$(pwd)":/ws -w /ws $(SIM_MSGS_IMAGE) \
		bash -c 'source /opt/ros/jazzy/setup.bash && \
			colcon build --base-paths $(INTERFACES) \
				--build-base build_jazzy --install-base $(SIM_INSTALL)'

lint:
	$(PYTHON) -m ruff check .

lint-domain:
	$(PYTHON) -m ruff check hexagon hexagon_tests

typecheck:
	$(PYTHON) -m mypy hexagon driven_adapters driving_adapters configuration

typecheck-domain:
	$(PYTHON) -m mypy hexagon

# Source ROS 2 and the interfaces overlay when they are present, so the adapter
# integration tests actually run instead of skipping. Both are absent in CI and
# in the Docker image, where the domain tests run on their own.
test:
	@set -e; \
	if [ -f "$(ROS_SETUP)" ]; then . "$(ROS_SETUP)"; fi; \
	if [ -f "$(OVERLAY_SETUP)" ]; then . "$(OVERLAY_SETUP)"; fi; \
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(PYTHON) -m pytest --tb=short || [ $$? -eq 5 ]

test-domain:
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(PYTHON) -m pytest hexagon_tests/ --tb=short

# Launch the Isaac Sim scene and submit one order in a single command. The
# scene takes 30-60s to load, so we wait for its subscriber to appear before
# submitting -- otherwise the order would be published before anything is
# listening and the arm would not move. Override ORDER=... to change the items.
run:
	@if [ ! -x "$(ISAAC_SIM)/python.sh" ]; then \
		echo "Isaac Sim not found at $(ISAAC_SIM). Set ISAAC_SIM=/path/to/isaacsim" >&2; exit 1; fi
	@if [ ! -f "$(ROS_SETUP)" ]; then \
		echo "ROS 2 setup not found at $(ROS_SETUP). Set ROS_SETUP=/path/to/setup.bash" >&2; exit 1; fi
	@if [ ! -f "$(OVERLAY_SETUP)" ]; then \
		echo "Custom messages not built. Run: make interfaces" >&2; exit 1; fi
	@if [ ! -d "$(SIM_INSTALL)" ]; then \
		echo "Simulator messages not built. Run: make interfaces-sim" >&2; exit 1; fi
	@set -e; \
	echo "Starting Isaac Sim scene (this can take 30-60s to load; the very"; \
	echo "first run is far slower while shaders are compiled and cached)..."; \
	ISAAC_SIM="$(ISAAC_SIM)" SIM_INSTALL="$(SIM_INSTALL)" $(SCENE) & \
	SCENE_PID=$$!; \
	trap 'kill $$SCENE_PID 2>/dev/null' EXIT; \
	. "$(ROS_SETUP)"; \
	. "$(OVERLAY_SETUP)"; \
	CLI_PYTHON=$$([ -x "$(PYTHON)" ] && echo "$(PYTHON)" || echo python3); \
	echo "Waiting for the scene to subscribe to $(TOPIC)..."; \
	PYTHONPATH="$$(pwd):$$PYTHONPATH" $$CLI_PYTHON simulation/wait_for_scene.py 180 || { \
		echo "Timed out waiting for the scene." >&2; exit 1; }; \
	echo "Scene ready. Submitting order: $(ORDER)"; \
	PYTHONPATH="$$(pwd):$$PYTHONPATH" $$CLI_PYTHON -m kinematic_kitchen submit "$(ORDER)"; \
	echo "Order submitted. Scene is running -- press Ctrl+C to stop."; \
	wait $$SCENE_PID

docker-build:
	docker build -t kinematic-kitchen .

docker-test:
	docker run --rm kinematic-kitchen
