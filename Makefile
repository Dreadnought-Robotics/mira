.PHONY: all check-uv check-ros build repoversion docker-ensure docker-x11 build-docker-container docker-fix-perms build-in-docker docker b install-deps install-mavproxy shell proxy-pixhawk get-submodules force-update install-udev fix-vscode validate-all camera_bottomcam camera_auto camera_frontcam alt_master alt_master_sitl teleop setup clean help

export FORCE_COLOR=1
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT={severity} {message}
export MACHINE_IP=$(shell hostname -I | awk '{print $$1}')
export OPENCV_FFMPEG_CAPTURE_OPTIONS="fifo_size;500000|overrun_nonfatal;1|fflags;nobuffer|flags;low_delay|framedrop;1|vf;setpts=0"
GSTREAMER_FIX=export LD_PRELOAD=$(shell gcc -print-file-name=libunwind.so.8 2>/dev/null || echo "libunwind.so.8")

ifeq ($(MACHINE_IP),192.168.2.6)
export MACHINE_NAME=ORIN
else ifeq ($(MACHINE_IP),192.168.2.4)
export MACHINE_NAME=RPI4
endif

SHELL := /bin/bash

WS := source .venv/bin/activate && source install/setup.bash

# Check if commands/directories exist at parse time
UV_EXISTS := $(shell command -v uv 2>/dev/null)
VENV_EXISTS := $(wildcard .venv)
ROS_JAZZY_EXISTS := $(wildcard /opt/ros/jazzy)
MAVPROXY_EXISTS := $(shell command -v mavproxy.py 2>/dev/null)$(shell command -v mavproxy 2>/dev/null)

all: build

# Resolve python paths
PYTHON3_PATH   := $(shell command -v python3 2>/dev/null)
PYTHON312_PATH := $(shell command -v python3.12 2>/dev/null)

check-uv:
ifndef UV_EXISTS
	$(error ❌ uv is not installed. Install it with: curl -LsSf https://astral.sh/uv/install.sh | sh)
endif

ifndef VENV_EXISTS
	$(warning ⚠️  Python virtual environment not found at .venv. Run make setup or uv sync to make it)
else
	$(info ✅ Virtual environment found at .venv.)
endif

# ---- Python checks ----
ifeq ($(PYTHON3_PATH),)
	$(error ❌ python3 not found in PATH)
endif

ifeq ($(PYTHON312_PATH),)
	$(error ❌ python3.12 not found in PATH)
endif

# Only block ~/.local/bin/python3, explicitly allow /usr/bin/python3 AND local .venv/bin/python3
ifeq ($(findstring .local/bin/python,$(PYTHON3_PATH)),.local/bin/python)
	$(error ❌ python3 resolves to ~/.local/bin. Please ensure you are using /usr/bin/python3 or the local .venv)
endif

$(info ✅ python3     → $(PYTHON3_PATH))
$(info ✅ python3.12  → $(PYTHON312_PATH))

check-ros: check-uv
ifndef ROS_JAZZY_EXISTS
	$(error ❌ ROS Jazzy not found at /opt/ros/jazzy. Only ROS Jazzy is supported by this workspace.)
endif
	$(info ✅ ROS Jazzy found.)

# Build the workspace
LINKER=lld 
CMAKE_ARGS:= -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
			 -DCMAKE_COLOR_DIAGNOSTICS=ON \
			 -GNinja \
			 -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=$(LINKER) \
			 -DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld=$(LINKER) \
			 -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=$(LINKER) \
			 --no-warn-unused-cli

SKIP_PACKAGES ?= vision_boundingbox vision_depth
export COLCON_ARGS= --cmake-args $(CMAKE_ARGS) \
                          --parallel-workers $(shell nproc) \
			  --packages-skip $(SKIP_PACKAGES) \
			  --event-handlers console_cohesion+

build: check-ros
	$(warning If you built in docker last - you'll need to clean and rebuild)
	$(warning If build fails b/c of CMakeCacheList or issues with mismatch for build,log,install, run \`make clean\`)
	$(info Building workspace...)
	@source /opt/ros/jazzy/setup.bash && \
	source .venv/bin/activate && \
	colcon build ${COLCON_ARGS}

repoversion:
	$(info Last commit in repository:)
	@git log -1 --oneline

docker-ensure:
	docker-compose up --no-recreate -d mira

docker-x11:
	xhost +local:docker || true

build-docker-container: docker-ensure
	$(info Building Docker container...)
	@docker-compose build mira

docker-fix-perms:
	sudo chown -R $(shell id -u):$(shell id -g) .

export _UID=$(shell id -u)
export _GID=$(shell id -g)
build-in-docker: docker-fix-perms docker-ensure
	docker-compose exec mira bash -c "make repoversion && make clean && make build"

docker: docker-ensure docker-x11
	docker-compose exec -u root mira /bin/bash

b: check-ros
ifndef P
	$(error No package specified. Please set P=package_name (e.g. make b P=mira2_control_master))
endif
	@source /opt/ros/jazzy/setup.bash && \
	source .venv/bin/activate && \
	colcon build ${COLCON_ARGS} --packages-select ${P}

install-deps: check-ros check-uv
	$(info ROS2 Jazzy, UV and Rosdep should be installed)
	$(info Installing basic build dependencies)
	@sudo apt-get update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y lld ninja-build build-essential cmake ros-jazzy-rmw-cyclonedds-cpp
	$(info Installing Python dependencies...)
	@[ -d .venv ] || uv venv --system-site-packages
	@uv sync
	$(info Installing ROS dependencies...)
	@source /opt/ros/jazzy/setup.bash && \
	rosdep install --from-paths src --ignore-src -r -y

PYTHON_VERSION ?= python3.12
install-mavproxy: check-uv
	$(info Installing mavproxy)
	@uv tool install mavproxy
	$(info Applying patch for mavproxy)
	@find ~/.local/share/uv/tools/mavproxy -name "rline.py" -exec patch {} < ./misc/patches/mavproxy_rline_fix.patch \;

shell:
	@bash --rcfile <(echo "cd $(CURDIR) && source $$HOME/.bashrc && source $(CURDIR)/.venv/bin/activate && source $(CURDIR)/install/setup.bash") -i

proxy-pixhawk:
	$(info If you get a realine error -> Edit the file mentioned in the stacktrace and remove the import from __future__ for input())
ifndef LAPTOP_IP
	$(error No LAPTOP_IP set. Set it like this: make proxy-pixhawk LAPTOP_IP=192.168.2.XX)
endif
ifndef MAVPROXY_EXISTS
	$(error ❌ mavproxy not found in PATH. Install with 'make install-mavproxy'.)
endif
	@mavproxy.py --master=/dev/Pixhawk --baudrate 57600 --out udp:$(LAPTOP_IP):14550

get-submodules:
	$(info Updating git submodules...)
	@git submodule update --init --recursive

force-update:
	$(info Fetching latest changes from remote...)
	@git fetch origin
	@git reset --hard origin/$$(git rev-parse --abbrev-ref HEAD)

install-udev:
	$(info Installing udev rules...)
	@sudo cp misc/udev/96-mira.rules /etc/udev/rules.d/
	@sudo udevadm control --reload-rules
	@sudo udevadm trigger

fix-vscode:
	$(info Fixing VSCode settings paths...)
	@current_dir=$$(realpath .); \
	settings_file=".vscode/settings.json"; \
	if [ -f "$$settings_file" ]; then \
		sed -i "s|\"/home/.*/mira\"|\"$$current_dir\"|g" "$$settings_file"; \
		echo "✅ Updated paths in $$settings_file"; \
	else \
		echo "⚠️  settings.json not found in .vscode directory."; \
	fi

validate-all:
	find ./src -type f -name "package.xml" -exec uv run ./util/package-utils/validate_package.py {} \;

camera_bottomcam:
	${WS} && ${GSTREAMER_FIX} && ros2 launch mira2_perception camera_imx335.launch camera_name:=camera_bottomcam

camera_auto:
	${WS} && ${GSTREAMER_FIX} && ros2 launch mira2_perception camera_auto.launch

camera_frontcam:
	${WS} && ${GSTREAMER_FIX} && ros2 launch mira2_perception camera_imx335.launch camera_name:=camera_frontcam

PIXHAWK_PORT ?= /dev/Pixhawk
alt_master: check-ros
	${WS} && ros2 launch mira2_control_master alt_master.launch pixhawk_address:=${PIXHAWK_PORT}

alt_master_sitl:
	$(info "Assuming Ardupilot SITL to running on same IP as THIS device with port 5760")
	${WS} && ros2 run mira2_control_master alt_master --ros-args -p pixhawk_address:=tcp:127.0.0.1:5760

teleop: check-ros
	${WS} && ros2 launch mira2_rov teleop.launch

setup: check-ros install-deps get-submodules build install-udev fix-vscode
	$(info 🚀 Complete workspace setup finished!)

clean:
	$(info Cleaning build artifacts...)
	@rm -rf build/ install/ log/
	$(info Clean completed.)

help:
	$(info Available targets:)
	$(info   build         - Build the ROS workspace)
	$(info   shell         - Source the workspace environment in a new shell)
	$(info   install-deps  - Install ROS & Python dependencies)
	$(info   get-submodules- Update git submodules)
	$(info   proxy-pixhawk - Proxy Pixhawk telemetry (Requires LAPTOP_IP=192.168.2.XX))
	$(info   force-update  - Get latest changes from remote)
	$(info   install-udev  - Install udev rules)
	$(info   b             - Build specific package (Requires P=package_name))
	$(info   fix-vscode    - Update local paths in .vscode settings)
	$(info   setup         - Complete workspace setup)
	$(info   clean         - Clean build artifacts)
	$(info )
	$(info ROS Launch targets:)
	$(info   alt_master      - Launch alternative master control)
	$(info   alt_master_sitl - Launch master control bound to SITL)
	$(info   teleop          - Launch teleoperation)
	$(info   camera_auto     - Launch perception node for auto camera)