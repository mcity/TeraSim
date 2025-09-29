#!/bin/bash
# TeraSim Monorepo Environment Setup Script (improved from original version)

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_command() {
    if ! command -v $1 &> /dev/null; then
        return 1
    fi
    return 0
}

check_python() {
    log_info "Checking Python environment..."
    if check_command python3; then
        PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
        log_info "Python version: $PYTHON_VERSION"

        # Check if Python version meets requirements
        PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
        PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)

        if [ "$PYTHON_MAJOR" -lt 3 ] || ([ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 10 ]); then
            log_error "Python 3.10+ required, found $PYTHON_VERSION"
            exit 1
        fi
    else
        log_error "Python3 not found. Please install Python 3.10 or higher"
        exit 1
    fi
}

check_gcc_gpp() {
    log_info "Checking for gcc and g++ compilers..."
    if ! check_command gcc; then
        log_error "gcc compiler not found. Please install gcc before proceeding."
        exit 1
    fi
    if ! check_command g++; then
        log_error "g++ compiler not found. Please install g++ before proceeding."
        exit 1
    fi
    log_info "gcc version: $(gcc --version | head -n1)"
    log_info "g++ version: $(g++ --version | head -n1)"
}


check_redis() {
    log_info "Checking Redis service..."
    if ! check_command redis-cli; then
        log_warning "Redis not installed"
        read -p "Install Redis? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            if [[ "$OSTYPE" == "linux-gnu"* ]]; then
                sudo apt-get update
                sudo apt-get install -y redis-server
            elif [[ "$OSTYPE" == "darwin"* ]]; then
                # macOS
                if check_command brew; then
                    brew install redis
                else
                    log_error "Please install Homebrew first or install Redis manually"
                    exit 1
                fi
            else
                log_error "Please install Redis manually: https://redis.io/download"
                exit 1
            fi
        fi
    fi
    
    # Check Redis service status (compatible with different systems)
    if command -v systemctl &> /dev/null; then
        if ! systemctl is-active --quiet redis-server 2>/dev/null; then
            log_warning "Redis service not running"
            read -p "Start Redis service? (y/n) " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                sudo systemctl start redis-server
                log_info "Redis service started"
            fi
        else
            log_info "Redis service is running"
        fi
    else
        # macOS or other systems
        if ! redis-cli ping &>/dev/null; then
            log_warning "Redis not responding. Please start Redis manually"
            if [[ "$OSTYPE" == "darwin"* ]]; then
                log_info "On macOS, you can start Redis with: brew services start redis"
            fi
        else
            log_info "Redis is responding"
        fi
    fi
}

setup_monorepo() {
    log_info "Setting up TeraSim monorepo..."

    # Initialize workspace and install dependencies
    log_info "Installing workspace packages..."
    # Install packages explicitly for conda compatibility
    pip install -e packages/terasim
    pip install -e packages/terasim-nde-nade
    pip install -e packages/terasim-service
    pip install -e packages/terasim-envgen
    pip install -e packages/terasim-datazoo
    pip install -e packages/terasim-vis
    pip install -e packages/terasim-cosmos

    # Install development dependencies
    pip install "pytest>=7.4.0" "pytest-cov>=4.1.0" "black>=23.7.0" "ruff>=0.1.0" "mypy>=1.5.1" "isort>=5.12.0"

    # Build Cython extensions
    if [ -d "packages/terasim-nde-nade" ]; then
        log_info "Building Cython extensions for NDE-NADE..."
        cd packages/terasim-nde-nade
        python setup.py build_ext --inplace
        cd ../..
    fi

    # Verify installation
    log_info "Testing installation..."
    python -c "
import terasim
print('✅ TeraSim core imported successfully')

try:
    import terasim_nde_nade
    print('✅ TeraSim NDE-NADE imported successfully')
except ImportError:
    print('⚠️  TeraSim NDE-NADE not available (optional)')

try:
    import terasim_service
    print('✅ TeraSim Service imported successfully')
except ImportError:
    print('⚠️  TeraSim Service not available (optional)')

try:
    import terasim_vis
    print('✅ TeraSim Visualization imported successfully')
except ImportError:
    print('⚠️  TeraSim Visualization not available (optional)')

try:
    import terasim_cosmos
    print('✅ TeraSim Cosmos imported successfully')
except ImportError:
    print('⚠️  TeraSim Cosmos not available (optional)')

print(f'TeraSim version: 0.2.0')
"
}

setup_sumo_tools() {
    log_info "Setting up SUMO tools..."
    
    # Create dependencies directory
    DEPS_DIR="${HOME}/.terasim/deps"
    mkdir -p "${DEPS_DIR}"
    
    # Set SUMO_HOME path
    SUMO_HOME="${DEPS_DIR}/sumo"
    
    if [ ! -d "${SUMO_HOME}" ]; then
        log_info "📦 Cloning SUMO repository for tools..."
        if ! check_command git; then
            log_error "Git not found. Please install git first"
            exit 1
        fi
        
        git clone --depth 1 https://github.com/eclipse/sumo.git "${SUMO_HOME}"
        log_info "✅ SUMO tools downloaded successfully"
    else
        log_info "🔄 Updating SUMO tools..."
        cd "${SUMO_HOME}"
        git pull origin main || git pull origin master || log_warning "Failed to update SUMO tools (not critical)"
        cd - > /dev/null
        log_info "✅ SUMO tools updated"
    fi
    
    # Export SUMO_HOME for current session
    export SUMO_HOME="${SUMO_HOME}"
    
    # Save SUMO_HOME path for reference
    echo "SUMO_HOME=${SUMO_HOME}" > "${DEPS_DIR}/.sumo_home"
    
    log_info "📍 SUMO_HOME: ${SUMO_HOME}"
    log_info "📍 SUMO tools: ${SUMO_HOME}/tools"
}

create_output_directories() {
    log_info "Creating output directories..."
    mkdir -p outputs logs
    log_info "Output directories created"
}

setup_environment_variables() {
    log_info "Setting up environment variables..."
    
    # Get SUMO_HOME from the saved file
    DEPS_DIR="${HOME}/.terasim/deps"
    if [ -f "${DEPS_DIR}/.sumo_home" ]; then
        source "${DEPS_DIR}/.sumo_home"
        log_info "📍 Found SUMO_HOME: ${SUMO_HOME}"
    else
        log_warning "SUMO_HOME not found. Please run SUMO setup first."
        return 1
    fi
    
    # Ask user if they want to add environment variables to shell config
    echo
    log_info "Environment variable setup options:"
    echo "1. Add to ~/.bashrc (recommended for bash users)"
    echo "2. Add to ~/.profile (recommended for all shells)"
    echo "3. Add to ~/.zshrc (for zsh users)"
    echo "4. Skip (environment variables will only be available in current session)"
    echo
    read -p "Choose an option (1-4): " -n 1 -r
    echo
    
    case $REPLY in
        1)
            SHELL_CONFIG="${HOME}/.bashrc"
            ;;
        2)
            SHELL_CONFIG="${HOME}/.profile"
            ;;
        3)
            SHELL_CONFIG="${HOME}/.zshrc"
            ;;
        4)
            log_info "Skipping persistent environment variable setup"
            log_info "To use SUMO in other sessions, run: source ${DEPS_DIR}/.sumo_home"
            return 0
            ;;
        *)
            log_warning "Invalid option. Skipping persistent environment variable setup"
            return 0
            ;;
    esac
    
    # Check if SUMO_HOME is already in the config file
    if grep -q "SUMO_HOME=" "${SHELL_CONFIG}" 2>/dev/null; then
        log_warning "SUMO_HOME already exists in ${SHELL_CONFIG}"
        read -p "Update existing SUMO_HOME? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            # Remove existing SUMO_HOME lines
            sed -i '/SUMO_HOME=/d' "${SHELL_CONFIG}"
        else
            log_info "Keeping existing SUMO_HOME configuration"
            return 0
        fi
    fi
    
    # Add environment variables to shell config
    echo "" >> "${SHELL_CONFIG}"
    echo "# TeraSim environment variables" >> "${SHELL_CONFIG}"
    echo "export SUMO_HOME=\"${SUMO_HOME}\"" >> "${SHELL_CONFIG}"
    echo "export PATH=\"\$SUMO_HOME/bin:\$PATH\"" >> "${SHELL_CONFIG}"
    
    log_info "✅ Environment variables added to ${SHELL_CONFIG}"
    log_info "📍 SUMO_HOME: ${SUMO_HOME}"
    log_info "📍 PATH updated to include SUMO tools"
    
    # Verify the setup
    log_info "Verifying environment variable setup..."
    if [ -f "${SUMO_HOME}/bin/sumo" ] || [ -f "${SUMO_HOME}/tools/sumo" ]; then
        log_info "✅ SUMO tools found in ${SUMO_HOME}"
    else
        log_warning "⚠️  SUMO tools not found in expected location"
    fi
    
    echo
    log_info "To apply changes in current session, run:"
    log_info "  source ${SHELL_CONFIG}"
    log_info "Or restart your terminal"
}


main() {
    log_info "Starting TeraSim monorepo setup..."
    
    check_python
    check_gcc_gpp
    check_redis
    setup_sumo_tools
    setup_environment_variables
    setup_monorepo
    create_output_directories
    
    log_info "🎉 Setup complete!"
    echo
    echo "TeraSim monorepo installation finished!"
    echo
    echo "Package installation status:"
    
    # Check each package import
    python -c "
import sys
packages = [
    ('terasim', 'Core simulation platform'),
    ('terasim_nde_nade', 'Neural differential equations enhancement'),
    ('terasim_vis', 'Visualization tools'),
    ('terasim_envgen', 'Environment generation tools'),
    ('terasim_datazoo', 'Data processing tools'),
    ('terasim_service', 'Service API'),
    ('terasim_cosmos', 'Cosmos-Drive integration')
]

for pkg_name, description in packages:
    try:
        __import__(pkg_name)
        print(f'  ✅ {pkg_name:<18} - {description}')
    except ImportError:
        print(f'  ❌ {pkg_name:<18} - {description} (failed)')
"
    
    echo
    echo "Environment variables:"
    echo "  SUMO_HOME is set to: ${SUMO_HOME:-'Not set'}"
    echo "  To verify: echo \$SUMO_HOME"
    echo
    echo "Development commands:"
    echo "  pytest                                 # Run tests"
    echo "  black .                                # Format code"
    echo "  python                                 # Start Python shell"
    echo
    echo "If SUMO_HOME is not set in new terminal sessions:"
    echo "  source ~/.terasim/deps/.sumo_home     # Load environment variables"
    echo
}

# Run main function only if script is executed directly
if [ "${BASH_SOURCE[0]}" == "${0}" ]; then
    main "$@"
fi