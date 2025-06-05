#
# Author: Yoshiki Obinata
#

# Makefile for testing on local

DOCKERFILES := $(wildcard tests/docker/*.Dockerfile)
DOCKER_TARGETS := $(patsubst %.Dockerfile, %, $(notdir $(DOCKERFILES)))

test: $(DOCKER_TARGETS)

$(DOCKER_TARGETS): %: tests/docker/%.Dockerfile
	@echo "=== Building Dockerfile: $< -> image: $@ ==="
	docker build -f $< .

.PHONY: test $(DOCKER_TARGETS)
