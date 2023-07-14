TARGET_HOST ?= homepi
TARGET_USERNAME ?= pi
TARGET_HOST_USER ?= $(TARGET_USERNAME)@$(TARGET_HOST)

.PHONY: build-docker
build-docker:
	rm -rf docker_out
	mkdir docker_out
	DOCKER_BUILDKIT=1 docker build --tag rplidar-zenoh-driver-builder --file Dockerfile --output type=local,dest=docker_out .

.PHONY: push-docker-built
push-docker-built: build-docker
	rsync -avz --delete docker_out/* $(TARGET_HOST_USER):/home/$(TARGET_USERNAME)/rplidar-zenoh-driver
