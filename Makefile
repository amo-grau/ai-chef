# Recipes source simulation/ros_env.sh, which uses bash-only syntax, so run them
# under bash rather than the default /bin/sh (dash).
SHELL := /bin/bash

VENV := .venv
PYTHON := $(VENV)/bin/python
PIP := $(VENV)/bin/pip

# Overridable locations for the live `run` target.
ISAAC_SIM ?= $(HOME)/isaacsim
SCENE := simulation/run_scene.sh
ROS_ENV := simulation/ros_env.sh
TOPIC := /kinematic_kitchen/prepare_order
ORDER ?= patty,bun,sauce

# The whole project speaks one ROS 2 distro: the Jazzy build Isaac Sim bundles,
# on Isaac Sim's Python 3.12. Jazzy is itself a Python 3.12 distro, so the
# generated PrepareOrder bindings are built once and used by the scene, the CLI
# and the ROS-dependent tests alike. See simulation/ros_env.sh.
INTERFACES := simulation/interfaces
INTERFACES_IMAGE ?= ros:jazzy-ros-base
INTERFACES_INSTALL := install_jazzy

.PHONY: install interfaces lint lint-domain typecheck typecheck-domain test test-domain run submit docker-build docker-test

# --system-site-packages lets the venv see apt-installed dev tools. The venv is
# for linting, typing and the domain tests only; anything that imports rclpy
# runs on Isaac Sim's Python instead (see ROS_ENV).
$(VENV):
	python3 -m venv --system-site-packages $(VENV)

install: $(VENV)
	$(PIP) install -e ".[dev]" -q

# Regenerate the custom message bindings. Re-run after editing any .msg file.
# Built inside a Jazzy container so no ROS 2 install is needed on the host; the
# result is loaded by Isaac Sim's Python 3.12, which is the same minor version.
interfaces:
	@if ! docker info >/dev/null 2>&1; then \
		echo "Docker is required to build the messages (image: $(INTERFACES_IMAGE))." >&2; exit 1; fi
	docker run --rm -u $$(id -u):$$(id -g) -v "$$(pwd)":/ws -w /ws $(INTERFACES_IMAGE) \
		bash -c 'source /opt/ros/jazzy/setup.bash && \
			colcon build --base-paths $(INTERFACES) \
				--build-base build_jazzy --install-base $(INTERFACES_INSTALL)'

lint:
	$(PYTHON) -m ruff check .

lint-domain:
	$(PYTHON) -m ruff check hexagon hexagon_tests

typecheck:
	$(PYTHON) -m mypy hexagon driven_adapters driving_adapters configuration

typecheck-domain:
	$(PYTHON) -m mypy hexagon

# Run on Isaac Sim's Python when it is available, so the adapter integration
# tests exercise rclpy instead of skipping; fall back to the venv otherwise.
# CI and the Docker image have neither, and run the domain tests on their own.
test:
	@set -e; \
	if . $(ROS_ENV) 2>/dev/null; then \
		echo "Running the full suite on Isaac Sim's Python ($$ROS_DISTRO)..."; \
		PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$$KK_PYTHON" -m pytest --tb=short || [ $$? -eq 5 ]; \
	else \
		echo "ROS 2 environment unavailable; running domain tests only." >&2; \
		PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(PYTHON) -m pytest hexagon_tests/ --tb=short || [ $$? -eq 5 ]; \
	fi

test-domain:
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(PYTHON) -m pytest hexagon_tests/ --tb=short

# Launch the Isaac Sim scene and submit one order in a single command. The
# scene takes 30-60s to load, so we wait for its subscriber to appear before
# submitting -- otherwise the order would be published before anything is
# listening and the arm would not move. Override ORDER=... to change the items.
run:
	@set -e; \
	. $(ROS_ENV); \
	echo "Starting Isaac Sim scene (this can take 30-60s to load; the very"; \
	echo "first run is far slower while shaders are compiled and cached)..."; \
	$(SCENE) & \
	SCENE_PID=$$!; \
	trap 'kill $$SCENE_PID 2>/dev/null' EXIT; \
	echo "Waiting for the scene to subscribe to $(TOPIC)..."; \
	"$$KK_PYTHON" simulation/wait_for_scene.py 180 || { \
		echo "Timed out waiting for the scene." >&2; exit 1; }; \
	echo "Scene ready. Submitting order: $(ORDER)"; \
	"$$KK_PYTHON" -m kinematic_kitchen submit "$(ORDER)"; \
	echo "Order submitted. Scene is running -- press Ctrl+C to stop."; \
	wait $$SCENE_PID

# Submit an order to an already-running scene. Override ORDER=... to change it.
submit:
	@set -e; \
	. $(ROS_ENV); \
	"$$KK_PYTHON" -m kinematic_kitchen submit "$(ORDER)"

docker-build:
	docker build -t kinematic-kitchen .

docker-test:
	docker run --rm kinematic-kitchen
