# DroneProgram developer entry points.
#
# `make check` is the gate. It is deliberately made of small, individually
# runnable targets, each of which encodes a regression this codebase actually
# had - so a failure names the specific mistake rather than "the build broke".

.DEFAULT_GOAL := help
SHELL := /bin/bash
PY := python3

.PHONY: help check test test-fast lint import-check no-shadowed-config \
        docs-check schema-check blocking-check repo-checks install preflight \
        sim fleet stop clean

help: ## Show this help
	@grep -hE '^[a-z-]+:.*?## ' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "} {printf "  \033[36m%-22s\033[0m %s\n", $$1, $$2}'

check: lint import-check repo-checks test ## Run every gate
	@echo
	@echo "make check: all gates green"

install: ## Editable install plus dev extras
	@# --no-build-isolation because this host's system setuptools (59.6)
	@# predates PEP 660, and pip's isolated build picks it up and refuses.
	@# --user is skipped inside a virtualenv (e.g. vision_env): a venv's own
	@# site-packages already has sys.path precedence over anything in
	@# ~/.local, so pip refuses --user there with "will lack sys.path
	@# precedence to setuptools in .../site-packages" instead of silently
	@# installing somewhere that would not shadow the old one.
	$(PY) -m pip install $$($(PY) -c 'import sys; print("" if sys.prefix != sys.base_prefix else "--user")') "setuptools>=68,<80"
	$(PY) -m pip install -e ".[dev]" --no-build-isolation

test: ## Full test suite (bench only; sitl/hitl excluded by pytest.ini)
	./scripts/test.sh

test-fast: ## Skip the tests that wait out real timeouts
	./scripts/test.sh -m "not slow"

lint: ## ruff check + format --check (fails the build)
	@if ! $(PY) -m ruff --version >/dev/null 2>&1; then \
	  echo "  ruff not installed - run: make install"; exit 1; \
	fi
	@$(PY) -m ruff check .
	@$(PY) -m ruff format --check .
	@echo "  lint: clean"

import-check: ## Every package must import from a neutral working directory
	@# THE EXACT CHECK THAT WOULD HAVE CAUGHT THE get_state_snapshot BLOCKER.
	@# fleet_dispatch/app.py imported a symbol that did not exist; because the
	@# import was inside a function, nothing failed until a dispatch arrived -
	@# and then it wedged a drone instead of crashing.
	@cd /tmp && $(PY) -c "\
import config, sim_topics, avoider_node; \
import drone_agent.geo, drone_agent.contracts, drone_agent.mission; \
import drone_agent.navigation, drone_agent.landing, drone_agent.setpoint; \
import drone_agent.safety_supervisor, drone_agent.payload, drone_agent.battery; \
import drone_agent.udp_receiver, drone_agent.gz_client, drone_agent.px4_params; \
import drone_web.drone_logic, fleet_dispatch.app, fleet_dispatch.db; \
import global_planner.detour, obstacle_memory_service.app; \
import perception.vision_bridge, world.marker_models; \
from drone_web.drone_logic import execute_delivery, get_state_snapshot; \
print('  import-check: every module imports from /tmp')"

no-shadowed-config: ## config.py must be the only source of tunables
	@$(PY) scripts/repo_checks.py tunables

docs-check: ## Every referenced .md must exist
	@$(PY) scripts/repo_checks.py docs

schema-check: ## Sim assets must agree with config.py
	@$(PY) scripts/repo_checks.py schema

blocking-check: ## No blocking calls in the flight path (finding F5)
	@$(PY) scripts/repo_checks.py blocking

repo-checks: ## All repository consistency gates at once
	@$(PY) scripts/repo_checks.py

preflight: ## Verify the running simulator before dispatching
	$(PY) scripts/preflight.py

sim: ## Launch Gazebo + one PX4 instance
	world/spawn_fleet.sh 1

fleet: ## Launch Gazebo + config.FLEET_SIZE PX4 instances
	world/spawn_fleet.sh

stop: ## Stop everything
	scripts/stop_system.sh

clean: ## Remove caches and runtime databases
	find . -name __pycache__ -type d -not -path './vision_env/*' -exec rm -rf {} +
	rm -rf .pytest_cache .ruff_cache *.egg-info
	rm -f fleet_dispatch/fleet.db* obstacle_memory_service/obstacles.db*
