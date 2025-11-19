-include .env
export
SHELL := /bin/bash
PWD := $(shell pwd)

CURRENT_DATE := $(shell date '+%Y_%m_%d_%H_%M_%S')

#IGNORE := $(shell bash -c "source /home/redacid/Projects/ESP32/setup-env.sh; env | sed 's/=/:=/' | sed 's/^/export /' > makeenv")
#include makeenv

# colors
GREEN = $(shell tput -Txterm setaf 2)
YELLOW = $(shell tput -Txterm setaf 3)
WHITE = $(shell tput -Txterm setaf 7)
RESET = $(shell tput -Txterm sgr0)
GRAY = $(shell tput -Txterm setaf 6)
TARGET_MAX_CHAR_NUM = 30

.EXPORT_ALL_VARIABLES:

.PHONY: all help clean build test

## Default target
all: help

makeenv:
	bash -c "source /home/redacid/Projects/ESP32/setup-env.sh; env | sed 's/=/:=/' | sed 's/^/export /' > makeenv"

## Show this help
help:
	@echo ''
	@echo 'Usage:'
	@echo '  ${YELLOW}make${RESET} ${GREEN}<target>${RESET}'
	@echo ''
	@echo 'Targets:'
	@awk '/^[a-zA-Z\-\_0-9]+:/ { \
		helpMessage = match(lastLine, /^## (.*)/); \
		if (helpMessage) { \
			helpCommand = substr($$1, 0, index($$1, ":")); \
			helpMessage = substr(lastLine, RSTART + 3, RLENGTH); \
			printf "  ${YELLOW}%-$(TARGET_MAX_CHAR_NUM)s${RESET} ${GREEN}%s${RESET}\n", helpCommand, helpMessage; \
		} \
	} \
	{ lastLine = $$0 }' $(MAKEFILE_LIST)

## Clean build directory
.ONESHELL:
clean:
	cd $(PWD)
	rm -rf build

## Build project with real ESPHome dependencies
.ONESHELL:
build: clean
	cd $(PWD)
	mkdir build
	cd build
	echo "Configuring with CMake..."
	cmake -DCMAKE_BUILD_TYPE=Debug ..
	echo "Building..."
	make -j$(nproc)

## Run tests
.ONESHELL:
test: build
	cd $(PWD)/build/test/
	echo "Running tests..."
	./rfm69_test

## Build and test
.ONESHELL:
all-test: test
	echo "Build and test completed!"