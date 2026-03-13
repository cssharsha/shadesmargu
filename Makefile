.PHONY: all compile_commands build run docker clean coverage setup-ci setup-ci-ssh clean-ci rerun

COMPOSE := docker compose -p shadesmar_dev -f docker/docker-compose.yml
SERVICE := shadesmar_gu

all: build

clean:
	$(COMPOSE) exec $(SERVICE) bazel clean --expunge

compile_commands:
	$(COMPOSE) exec $(SERVICE) bazel run @hedron_compile_commands//:refresh_all
	sed -i 's|"directory": "/workspace"|"directory": "$(CURDIR)"|g' compile_commands.json
	./scripts/generate_clangd_config.sh

build:
	$(COMPOSE) exec $(SERVICE) bazel build --config=cuda $(TARGET)

test:
	$(COMPOSE) exec $(SERVICE) bazel test --config=cuda $(if $(TARGET),$(TARGET),//...)

run:
	$(COMPOSE) exec $(SERVICE) bazel run --config=cuda $(TARGET) -- $(ARGS)

coverage:
	$(COMPOSE) exec $(SERVICE) bazel coverage --config=cuda \
		-- //subastral/backend:common_test \
		   //subastral/backend:jacobian_test \
		   //subastral/backend/lie:so3_test \
		   //subastral/backend/lie:se3_test \
		   //subastral/backend/solver:lm_solver_test \
		   //subastral/backend/solver:schur_test \
		   //subastral/backend/solver:loss_function_test \
		   //subastral/loader:g2o_loader_test
	@echo ""
	@echo "=== Generating HTML report ==="
	$(COMPOSE) exec $(SERVICE) bash -c ' \
		LCOV_FILE="$$(bazel info output_path)/_coverage/_coverage_report.dat"; \
		if [ -f "$$LCOV_FILE" ]; then \
			genhtml "$$LCOV_FILE" \
				--output-directory /workspace/coverage_report \
				--title "Shadesmar Coverage" \
				--legend 2>/dev/null; \
			echo ""; \
			lcov --summary "$$LCOV_FILE" 2>&1; \
			echo ""; \
			echo "HTML report: coverage_report/index.html"; \
		else \
			echo "No coverage report found"; \
		fi'

docker:
	$(COMPOSE) up -d --build

setup-ci:
	@./scripts/setup_buildkite_agent.sh

setup-ci-ssh:
	@echo "=== Copying SSH keys to buildkite-agent user ==="
	@SSH_KEY=$$(ls ~/.ssh/id_ed25519 2>/dev/null || ls ~/.ssh/id_rsa 2>/dev/null) && \
	if [ -z "$$SSH_KEY" ]; then \
		echo "Error: No SSH key found at ~/.ssh/id_ed25519 or ~/.ssh/id_rsa"; \
		exit 1; \
	fi && \
	echo "Found SSH key: $$SSH_KEY" && \
	sudo mkdir -p /var/lib/buildkite-agent/.ssh && \
	sudo cp "$$SSH_KEY" /var/lib/buildkite-agent/.ssh/ && \
	sudo cp "$$SSH_KEY.pub" /var/lib/buildkite-agent/.ssh/ 2>/dev/null || true && \
	sudo chown -R buildkite-agent:buildkite-agent /var/lib/buildkite-agent/.ssh && \
	sudo chmod 700 /var/lib/buildkite-agent/.ssh && \
	sudo chmod 600 /var/lib/buildkite-agent/.ssh/$$(basename $$SSH_KEY) && \
	echo "SSH key copied to buildkite-agent user" && \
	echo "Restarting buildkite-agent..." && \
	sudo systemctl restart buildkite-agent && \
	echo "Done. Retrigger the build from the Buildkite dashboard."

clean-ci:
	@echo "=== Fixing buildkite-agent home directory ==="
	@sudo rm -rf /var/lib/buildkite-agent/.gitconfig
	@echo -e "[init]\n\tdefaultBranch = main" | sudo tee /var/lib/buildkite-agent/.gitconfig > /dev/null
	@sudo chown buildkite-agent:buildkite-agent /var/lib/buildkite-agent/.gitconfig
	@echo "=== Clearing stale builds ==="
	@sudo rm -rf /var/lib/buildkite-agent/.buildkite-agent/builds/*
	@echo "=== Restarting agent ==="
	@sudo systemctl restart buildkite-agent
	@echo "Done. Retrigger the build from the Buildkite dashboard."

rerun:
	@$(COMPOSE) exec -T $(SERVICE) pkill -f 'rerun --serve-web' 2>/dev/null || true
	@sleep 1
	$(COMPOSE) exec -d $(SERVICE) rerun --serve-web
	@echo "Rerun web viewer: http://localhost:9090"
	@echo "gRPC proxy: rerun+http://localhost:9876/proxy"

docker-clean:
	$(COMPOSE) down
