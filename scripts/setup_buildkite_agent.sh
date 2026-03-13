#!/usr/bin/env bash
#
# Setup Buildkite Agent on the host machine for GPU CI.
#
# Detects the host OS (Ubuntu/Debian, Fedora/RHEL, Arch, macOS) and installs
# the Buildkite agent accordingly. Prompts for the agent token if not provided
# via the BUILDKITE_AGENT_TOKEN environment variable.
#
# Usage:
#   ./scripts/setup_buildkite_agent.sh
#   BUILDKITE_AGENT_TOKEN=xxx ./scripts/setup_buildkite_agent.sh
#
# Prerequisites:
#   - Docker + Docker Compose
#   - NVIDIA Container Toolkit (for GPU tests)
#   - sudo access

set -euo pipefail

# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
check_prerequisites() {
    info "Checking prerequisites..."

    if ! command -v docker &>/dev/null; then
        error "Docker is not installed. Install Docker first: https://docs.docker.com/engine/install/"
    fi

    if ! docker compose version &>/dev/null; then
        error "Docker Compose (v2) is not available. Install it: https://docs.docker.com/compose/install/"
    fi

    # Check for NVIDIA GPU support (warn, don't fail — user might be setting up
    # before installing the toolkit)
    if ! command -v nvidia-smi &>/dev/null; then
        warn "nvidia-smi not found. GPU tests require NVIDIA Container Toolkit."
        warn "Install: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html"
    else
        info "NVIDIA GPU detected: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
    fi

    # Check if agent is already installed
    if command -v buildkite-agent &>/dev/null; then
        warn "Buildkite agent is already installed: $(buildkite-agent --version)"
        read -rp "Reinstall/reconfigure? [y/N] " answer
        if [[ ! "$answer" =~ ^[Yy]$ ]]; then
            info "Exiting. To reconfigure, edit /etc/buildkite-agent/buildkite-agent.cfg"
            exit 0
        fi
    fi
}

# ---------------------------------------------------------------------------
# Detect OS
# ---------------------------------------------------------------------------
detect_os() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "macos"
        return
    fi

    if [[ -f /etc/os-release ]]; then
        . /etc/os-release
        case "$ID" in
            ubuntu|debian|linuxmint|pop)
                echo "debian"
                ;;
            fedora|rhel|centos|rocky|almalinux)
                echo "redhat"
                ;;
            arch|manjaro|endeavouros)
                echo "arch"
                ;;
            *)
                # Try ID_LIKE as fallback
                case "${ID_LIKE:-}" in
                    *debian*|*ubuntu*)
                        echo "debian"
                        ;;
                    *rhel*|*fedora*)
                        echo "redhat"
                        ;;
                    *arch*)
                        echo "arch"
                        ;;
                    *)
                        echo "unknown"
                        ;;
                esac
                ;;
        esac
    else
        echo "unknown"
    fi
}

# ---------------------------------------------------------------------------
# Get agent token
# ---------------------------------------------------------------------------
get_agent_token() {
    if [[ -n "${BUILDKITE_AGENT_TOKEN:-}" ]]; then
        info "Using agent token from BUILDKITE_AGENT_TOKEN environment variable"
        return
    fi

    # Try to read from scripts/secrets file
    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local secrets_file="$script_dir/secrets"

    if [[ -f "$secrets_file" ]]; then
        local token_line
        token_line=$(grep -E '^BUILDKITE=' "$secrets_file" 2>/dev/null || true)
        if [[ -n "$token_line" ]]; then
            BUILDKITE_AGENT_TOKEN="${token_line#BUILDKITE=}"
            info "Using agent token from scripts/secrets"
            export BUILDKITE_AGENT_TOKEN
            return
        fi
    fi

    echo ""
    echo "You need a Buildkite agent token. Either:"
    echo "  1. Add it to scripts/secrets as: BUILDKITE=<token>"
    echo "  2. Set BUILDKITE_AGENT_TOKEN env var"
    echo "  3. Get it from: Buildkite Dashboard → Settings → Agents → Reveal Agent Token"
    echo ""
    read -rp "Paste your agent token: " BUILDKITE_AGENT_TOKEN

    if [[ -z "$BUILDKITE_AGENT_TOKEN" ]]; then
        error "Agent token cannot be empty"
    fi

    export BUILDKITE_AGENT_TOKEN
}

# ---------------------------------------------------------------------------
# Install: Debian/Ubuntu
# ---------------------------------------------------------------------------
install_debian() {
    info "Installing Buildkite agent on Debian/Ubuntu..."

    sudo sh -c 'echo deb https://apt.buildkite.com/buildkite-agent stable main > /etc/apt/sources.list.d/buildkite-agent.list'
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
        --recv-keys 32A37959C2FA5C3C99EFBC32A79206696452D198 2>/dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq buildkite-agent
}

# ---------------------------------------------------------------------------
# Install: RedHat/Fedora
# ---------------------------------------------------------------------------
install_redhat() {
    info "Installing Buildkite agent on RedHat/Fedora..."

    sudo sh -c 'cat > /etc/yum.repos.d/buildkite-agent.repo <<EOF
[buildkite-agent]
name=Buildkite Pty Ltd
baseurl=https://yum.buildkite.com/buildkite-agent/stable/x86_64/
enabled=1
gpgcheck=0
priority=1
EOF'
    sudo yum install -y buildkite-agent
}

# ---------------------------------------------------------------------------
# Install: Direct binary (fallback for distros without good packages)
# ---------------------------------------------------------------------------
install_linux_binary() {
    info "Installing Buildkite agent from official binary release..."

    local version="3.119.2"
    local arch="amd64"
    local url="https://github.com/buildkite/agent/releases/download/v${version}/buildkite-agent-linux-${arch}-${version}.tar.gz"
    local install_dir="/usr/local/bin"
    local config_dir="/etc/buildkite-agent"

    # Download and extract
    local tmpdir
    tmpdir=$(mktemp -d)
    info "Downloading buildkite-agent v${version}..."
    curl -sL "$url" | tar xz -C "$tmpdir"

    # Install binary
    sudo install -m 755 "$tmpdir/buildkite-agent" "$install_dir/buildkite-agent"
    info "Installed buildkite-agent to $install_dir/buildkite-agent"

    # Create config directory and default config if needed
    sudo mkdir -p "$config_dir"
    if [[ ! -f "$config_dir/buildkite-agent.cfg" ]]; then
        sudo cp "$tmpdir/buildkite-agent.cfg" "$config_dir/buildkite-agent.cfg" 2>/dev/null || true
    fi

    # Create buildkite-agent user if it doesn't exist
    if ! id -u buildkite-agent &>/dev/null; then
        sudo useradd --system --home-dir /var/lib/buildkite-agent \
            --create-home --shell /bin/bash buildkite-agent
        info "Created buildkite-agent system user"
    fi

    # Add buildkite-agent user to docker group so it can run docker commands
    if getent group docker &>/dev/null; then
        sudo usermod -aG docker buildkite-agent
        info "Added buildkite-agent to docker group"
    else
        warn "docker group not found — buildkite-agent may not be able to run docker commands"
    fi

    # Create systemd service
    sudo tee /etc/systemd/system/buildkite-agent.service > /dev/null <<EOF
[Unit]
Description=Buildkite Agent
Documentation=https://buildkite.com/docs/agent/v3
After=network.target

[Service]
Type=simple
User=buildkite-agent
ExecStart=/usr/local/bin/buildkite-agent start --config /etc/buildkite-agent/buildkite-agent.cfg
RestartSec=5
Restart=on-failure
TimeoutStartSec=10
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload

    # Cleanup
    rm -rf "$tmpdir"

    info "Buildkite agent v${version} installed successfully"
}

# ---------------------------------------------------------------------------
# Install: Arch
# ---------------------------------------------------------------------------
install_arch() {
    info "Installing Buildkite agent on Arch Linux..."

    # Install directly from Buildkite's official Linux binary release.
    # The AUR package (buildkite-agent-bin) frequently has stale checksums,
    # so we use the official install script instead.
    install_linux_binary
}

# ---------------------------------------------------------------------------
# Install: macOS
# ---------------------------------------------------------------------------
install_macos() {
    info "Installing Buildkite agent on macOS..."

    if ! command -v brew &>/dev/null; then
        error "Homebrew is not installed. Install it first: https://brew.sh"
    fi

    brew install buildkite/buildkite/buildkite-agent
}

# ---------------------------------------------------------------------------
# Configure agent
# ---------------------------------------------------------------------------
configure_agent() {
    info "Configuring Buildkite agent..."

    local config_file=""

    if [[ "$1" == "macos" ]]; then
        config_file="$(brew --prefix)/etc/buildkite-agent/buildkite-agent.cfg"
    else
        # Search common locations
        for candidate in \
            /etc/buildkite-agent/buildkite-agent.cfg \
            /usr/share/buildkite-agent/buildkite-agent.cfg \
            /opt/buildkite-agent/buildkite-agent.cfg; do
            if [[ -f "$candidate" ]]; then
                config_file="$candidate"
                break
            fi
        done

        # If still not found, try to locate it
        if [[ -z "$config_file" ]]; then
            config_file=$(find /etc /usr/share /opt -name "buildkite-agent.cfg" 2>/dev/null | head -1 || true)
        fi
    fi

    if [[ -z "$config_file" || ! -f "$config_file" ]]; then
        # Create config directory and file if the package didn't
        info "Config file not found, creating /etc/buildkite-agent/buildkite-agent.cfg"
        sudo mkdir -p /etc/buildkite-agent
        sudo tee /etc/buildkite-agent/buildkite-agent.cfg > /dev/null <<EOF
# Buildkite Agent Configuration
# See: https://buildkite.com/docs/agent/v3/configuration
token="${BUILDKITE_AGENT_TOKEN}"
name="%hostname-%n"
build-path="/var/lib/buildkite-agent/builds"
EOF
        sudo mkdir -p /var/lib/buildkite-agent/builds
        config_file="/etc/buildkite-agent/buildkite-agent.cfg"
        info "Config created at $config_file"
        return
    fi

    # Set the agent token in existing config
    if grep -q '^token=' "$config_file"; then
        sudo sed -i.bak "s/^token=.*/token=\"${BUILDKITE_AGENT_TOKEN}\"/" "$config_file"
    else
        echo "token=\"${BUILDKITE_AGENT_TOKEN}\"" | sudo tee -a "$config_file" > /dev/null
    fi

    info "Agent token configured in $config_file"
}

# ---------------------------------------------------------------------------
# Start agent
# ---------------------------------------------------------------------------
start_agent() {
    info "Starting Buildkite agent..."

    if [[ "$1" == "macos" ]]; then
        brew services start buildkite/buildkite/buildkite-agent
        info "Agent started via Homebrew services"
    else
        sudo systemctl enable buildkite-agent
        sudo systemctl start buildkite-agent

        # Verify it's running
        sleep 2
        if sudo systemctl is-active --quiet buildkite-agent; then
            info "Agent is running"
        else
            warn "Agent may not have started. Check: sudo systemctl status buildkite-agent"
        fi
    fi
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
    echo ""
    echo "=========================================="
    echo "  Buildkite Agent Setup for Shadesmar CI"
    echo "=========================================="
    echo ""
    echo "This installs a Buildkite agent on your host machine."
    echo "The agent makes outbound-only HTTPS connections to Buildkite."
    echo "No inbound ports are opened."
    echo ""

    check_prerequisites

    local os
    os=$(detect_os)
    info "Detected OS family: $os"

    if [[ "$os" == "unknown" ]]; then
        error "Unsupported OS. Install the agent manually: https://buildkite.com/docs/agent/v3/installation"
    fi

    get_agent_token

    case "$os" in
        debian)  install_debian  ;;
        redhat)  install_redhat  ;;
        arch)    install_arch    ;;
        macos)   install_macos   ;;
    esac

    configure_agent "$os"
    start_agent "$os"

    echo ""
    info "Setup complete!"
    echo ""
    echo "  Agent token:  configured"
    echo "  Agent status: $(sudo systemctl is-active buildkite-agent 2>/dev/null || echo 'check manually')"
    echo ""
    echo "  Next steps:"
    echo "    1. Verify agent appears in Buildkite dashboard → Settings → Agents"
    echo "    2. Push a commit or open a PR to trigger a build"
    echo "    3. Monitor at: https://buildkite.com"
    echo ""
}

main "$@"
