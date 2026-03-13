.PHONY: all compile_commands build run docker clean

all: build

clean:
	docker compose -f docker/docker-compose.yml exec slam_course bazel clean --expunge

compile_commands:
	docker compose -f docker/docker-compose.yml exec slam_course bazel run @hedron_compile_commands//:refresh_all
	sed -i 's|"directory": "/workspace"|"directory": "$(CURDIR)"|g' compile_commands.json
	./generate_clangd_config.sh

build:
	docker compose -f docker/docker-compose.yml exec slam_course bazel build --config=cuda $(TARGET)

test:
	docker compose -f docker/docker-compose.yml exec slam_course bazel test --config=cuda $(if $(TARGET),$(TARGET),//...)

run:
	docker compose -f docker/docker-compose.yml exec slam_course bazel run --config=cuda $(TARGET) -- $(ARGS)

docker:
	docker compose -f docker/docker-compose.yml up -d --build

docker-clean:
	docker compose -f docker/docker-compose.yml down
