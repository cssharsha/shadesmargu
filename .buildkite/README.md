# Buildkite CI Setup

Self-hosted Buildkite agent running on a local GPU machine. The agent makes
outbound-only HTTPS connections to Buildkite's control plane — no inbound ports
are exposed on your machine.

## Architecture

```
GitHub push/PR
      │
      ▼
Buildkite.com (hosted control plane)
      │  outbound HTTPS poll
      ▼
Your GPU machine (self-hosted agent)
      │
      ▼
Docker container (CUDA 11.8 + Bazel 7.4.1)
      │
      ▼
Build → Test → Coverage → Cleanup
```

The agent polls Buildkite over HTTPS — no inbound ports, no tunnels, no public
IP required.

## Container Isolation

Local development and CI use **separate Docker Compose projects** so they never
interfere with each other:

| Context | Project name | Compose files | Container name |
|---------|-------------|---------------|----------------|
| Local dev | `shadesmar_dev` | `docker-compose.yml` | `shadesmar_dev-shadesmar_gu-1` |
| CI | `shadesmar_ci` | `docker-compose.yml` + `docker-compose.ci.yml` | `shadesmar_ci-shadesmar_gu-1` |

The CI override (`docker-compose.ci.yml`) runs the container as the
`buildkite-agent` user's UID/GID so all files created in `/workspace` are owned
by the agent. This eliminates checkout permission errors without needing chown
hacks, sudoers rules, or agent hooks.

## One-Time Setup

### 1. Create Buildkite Organization

1. Sign up at [buildkite.com](https://buildkite.com)
2. Create an organization
3. Go to **Settings → Connected Apps → GitHub**, authorize and select the
   `cssharsha/shadesmargu` repository

### 2. Create a Self-Hosted Queue

Buildkite's "Default cluster" defaults to hosted (cloud) agents. These have
**no GPU** — `docker compose` will fail with "could not select device driver
nvidia". You must create a self-hosted queue:

1. **Settings → Agents → Default cluster**
2. Click **Queues → New Queue**
3. Name: `arch-sh` (must match `agents: queue:` in `pipeline.yml`)
4. Type: **Self-Hosted**
5. Save

### 3. Create the Pipeline

1. **Pipelines → New Pipeline**
2. Name: `shadesmargu`, select the GitHub repo
3. In the **Steps** editor, replace the default with:
   ```yaml
   steps:
     - label: ":pipeline: Upload"
       command: "buildkite-agent pipeline upload .buildkite/pipeline.yml"
       agents:
         queue: "arch-sh"
   ```
4. Under **GitHub Settings**, enable:
   - "Build pull requests"
   - "Build pushes"
5. Save

### 4. Get the Agent Token

1. **Settings → Agents → Default cluster → Agent Tokens**
2. Reveal or create a token
3. Save it to `scripts/secrets` (this file is gitignored):
   ```
   BUILDKITE=<your-agent-token>
   ```

### 5. Install the Agent

Do **not** use the AUR `buildkite-agent-bin` package — its PKGBUILD has stale
sha256 checksums that fail verification. Install from the official binary
release instead:

```bash
make setup-ci
```

This runs `scripts/setup_buildkite_agent.sh` which:
- Detects your OS (Ubuntu/Debian, Fedora/RHEL, Arch, macOS)
- Reads the token from `scripts/secrets` automatically
- Downloads the official Buildkite agent binary (not from distro packages)
- Creates a `buildkite-agent` system user and adds it to the `docker` group
- Writes `/etc/buildkite-agent/buildkite-agent.cfg` with the token and
  `tags="queue=arch-sh"`
- Creates a systemd service and starts it

### 6. Grant the Agent SSH Access to GitHub

The `buildkite-agent` system user needs SSH keys to clone the repo:

```bash
make setup-ci-ssh
```

This copies your SSH key (`~/.ssh/id_ed25519` or `~/.ssh/id_rsa`) to
`/var/lib/buildkite-agent/.ssh/` and restarts the agent.

### 7. Fix the Agent's `.gitconfig`

The `buildkite-agent` user's home directory may not have a valid `.gitconfig`.
If git operations fail with "fatal: not in a git directory" or similar, run:

```bash
make clean-ci
```

This writes a minimal `.gitconfig`, clears stale build directories, and
restarts the agent.

### 8. Verify

```bash
# Check agent is running and connected
systemctl status buildkite-agent

# Should show:
#   Successfully registered agent "archlinux-1" with tags [queue=arch-sh]
#   Waiting for instructions...

# Live logs
journalctl -u buildkite-agent -f
```

The agent should also appear in the Buildkite dashboard under
**Settings → Agents**.

## Pipeline Steps

Defined in `.buildkite/pipeline.yml`. All steps run inside the project's Docker
container via `docker compose exec`:

| Step | What it does | Depends on | Timeout |
|------|-------------|------------|---------|
| Build | `bazel build --config=cuda //subastral/... //viz/...` | — | 30 min |
| Unit Tests | `bazel test --config=cuda //subastral/... //third_party/tests/...` | Build | 30 min |
| Code Coverage | `bazel coverage` on CPU test targets, generates lcov HTML report | Build | 20 min |
| Cleanup | `docker compose down` — stops the CI container | Tests + Coverage | 5 min |

Tests and coverage run in parallel after the build step completes. The cleanup
step runs after both finish (even if they fail) and tears down the CI container.

## CI Container Details

The CI pipeline uses a compose override (`docker/docker-compose.ci.yml`) that:

- **Runs as the host user's UID/GID** (`HOST_UID`/`HOST_GID` exported before
  `docker compose up`). All files in `/workspace` are owned by `buildkite-agent`.
- **Sets `HOME=/tmp/ci-home`** so the arbitrary UID can write caches without
  needing `/home/developer`.
- **Sets `USER=ci`** — Bazel requires `$USER` to be set, and the host UID
  doesn't exist in the container's `/etc/passwd`.
- **Uses ephemeral Bazel caches** in `/tmp/ci-home/.cache/` (no persistent
  volumes). Each container start gets a fresh cache.

This eliminates all permission issues between the container and the host
without needing chown, sudoers rules, or agent hooks.

## Coverage

Coverage is collected from CPU-compiled test targets only. CUDA `.cu` files
compiled by nvcc do not produce gcov data.

The Dockerfile replaces system `gcov` with an `llvm-cov-14 gcov` wrapper
(`dpkg-divert` + symlink) because gcc's gcov segfaults on clang-produced
`.gcno` files (gcov version `B14*` vs clang's `408*`).

Current results: ~95% line coverage, 100% function coverage.

**Branch coverage** is not yet supported. Bazel's LcovMerger does not correctly
parse `llvm-cov`'s intermediate branch format (`taken`/`nottaken`/`notexec`) —
all branches show as not executed in the lcov output. This requires either
lcov 2.0+ or switching to LLVM native coverage (`-fprofile-instr-generate`).

## Artifacts

Each build uploads:
- `bazel-testlogs/**/test.log` — individual test logs
- `bazel-testlogs/**/test.xml` — JUnit XML results
- `coverage_report/**/*` — HTML coverage report
- `coverage.lcov` — raw lcov data

## Local Equivalents

```bash
make build TARGET=//subastral/...    # Build
make test                            # All tests
make coverage                        # Coverage report → coverage_report/index.html
```

## Troubleshooting

### Docker: "could not select device driver nvidia"

The build is running on a Buildkite hosted agent (cloud) instead of your
self-hosted agent. Make sure:
1. Your self-hosted queue exists and matches `pipeline.yml`
2. The agent is running: `systemctl status buildkite-agent`
3. The pipeline upload step also has `agents: queue: "arch-sh"`

### Agent won't register: queue not found

The queue name in `pipeline.yml` must match the queue you created in Buildkite.
Check:

```bash
grep 'queue:' .buildkite/pipeline.yml
```

### Git clone fails: Permission denied (publickey)

The `buildkite-agent` user doesn't have SSH keys. Run:

```bash
make setup-ci-ssh
```

### Git fails: "fatal: not in a git directory"

The `buildkite-agent` user's `.gitconfig` is missing or corrupt (e.g., it was
accidentally created as a directory by `mkdir`). Run:

```bash
make clean-ci
```

### OCI: "container breakout detected"

`docker compose exec` inherits the host's CWD. If the host CWD doesn't exist
inside the container, Docker refuses with a "container breakout" error. The
pipeline uses `exec -w /workspace` (via the `EXEC` env var) to force the
container working directory.

### Bazel: "$USER is not set"

The CI container runs as an arbitrary UID that doesn't exist in `/etc/passwd`.
Bazel requires `$USER` to be set. The CI override sets `USER=ci` in the
environment.

### Coverage segfaults

If `gcov` segfaults during coverage collection, the system `gcov` (gcc) is
being used instead of the `llvm-cov-14` wrapper. Check:

```bash
docker compose -p shadesmar_dev -f docker/docker-compose.yml exec shadesmar_gu gcov --version
# Should show: "Ubuntu LLVM version 14.0.0"
# NOT: "gcov (Ubuntu 11.x.x) 11.x.x"
```

If it shows gcc's gcov, rebuild the Docker image (`make docker`).

### Buildkite YAML variable expansion

Buildkite expands `$VAR` in pipeline YAML. To use shell variables inside
`command:` blocks, escape with `$$`:

```yaml
command: |
  $${EXEC} -T $${SERVICE} bazel build ...
```

Single `$` will be expanded by Buildkite (and likely become empty).

### Agent config and logs

```bash
# Config
cat /etc/buildkite-agent/buildkite-agent.cfg

# Logs
journalctl -u buildkite-agent -f
```

## Files

```
.buildkite/
  pipeline.yml                Pipeline definition (build, test, coverage, cleanup)
  README.md                   This file
scripts/
  setup_buildkite_agent.sh    Agent installer (cross-platform, reads scripts/secrets)
  generate_coverage_report.sh HTML report generator (runs inside container)
  generate_clangd_config.sh   Host clangd config generator (runs on host)
  secrets                     Agent token (gitignored)
docker/
  Dockerfile                  Includes: bazelisk, lcov, llvm-cov gcov wrapper
  docker-compose.yml          Base compose (local dev, project: shadesmar_dev)
  docker-compose.ci.yml       CI override (runs as host UID, ephemeral caches)
```
