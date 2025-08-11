#!/bin/bash

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
    else
        log_error "Python3 not found. Please install Python 3.8 or higher"
        exit 1
    fi
}

check_poetry() {
    log_info "Checking Poetry..."
    if ! check_command poetry; then
        log_warning "Poetry not installed. Installing now..."
        curl -sSL https://install.python-poetry.org | python3 -
        export PATH="$HOME/.local/bin:$PATH"
        
        if ! check_command poetry; then
            log_error "Poetry installation failed. Please install manually: https://python-poetry.org/docs/#installation"
            exit 1
        fi
    fi
    log_info "Poetry version: $(poetry --version)"
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
            else
                log_error "Please install Redis manually: https://redis.io/download"
                exit 1
            fi
        fi
    fi
    
    if ! systemctl is-active --quiet redis-server; then
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
}

download_repositories() {
    log_info "Checking and downloading required repositories..."
    
    if [ -f "download_repo.sh" ]; then
        log_info "Running download_repo.sh..."
        bash download_repo.sh
    else
        log_warning "download_repo.sh not found. Checking repositories manually..."
        
        REPOS=(
            "TeraSim"
            "TeraSim-Service"
            "TeraSim-NDE-NADE"
        )
        
        for repo in "${REPOS[@]}"; do
            if [ ! -d "$repo" ]; then
                log_warning "$repo directory not found. Please clone manually or provide repository URL"
            else
                log_info "$repo directory exists"
            fi
        done
    fi
}

pull_latest_code() {
    log_info "Pulling latest code from repositories..."
    
    # First, pull the main repository (TeraSim-Deploy)
    log_info "Updating main repository (TeraSim-Deploy)..."
    if [ -d ".git" ]; then
        CURRENT_BRANCH=$(git branch --show-current)
        log_info "Current branch: $CURRENT_BRANCH"
        
        # Check for uncommitted changes
        if ! git diff-index --quiet HEAD --; then
            log_warning "Main repository has uncommitted changes. Skipping pull."
        else
            # Pull latest changes
            log_info "Pulling latest changes for main repository..."
            git pull origin "$CURRENT_BRANCH"
            
            if [ $? -eq 0 ]; then
                log_info "Main repository updated successfully"
            else
                log_error "Failed to update main repository"
            fi
        fi
    else
        log_warning "Current directory is not a git repository"
    fi
    
    # Then pull the submodules
    REPOS=(
        "TeraSim"
        "TeraSim-Service"
        "TeraSim-NDE-NADE"
    )
    
    for repo in "${REPOS[@]}"; do
        if [ -d "$repo" ]; then
            log_info "Updating $repo..."
            cd "$repo"
            
            # Check if it's a git repository
            if [ -d ".git" ]; then
                # Save current branch
                CURRENT_BRANCH=$(git branch --show-current)
                log_info "Current branch: $CURRENT_BRANCH"
                
                # Check for uncommitted changes
                if ! git diff-index --quiet HEAD --; then
                    log_warning "$repo has uncommitted changes. Skipping pull."
                    cd ..
                    continue
                fi
                
                # Pull latest changes
                log_info "Pulling latest changes for $repo..."
                git pull origin "$CURRENT_BRANCH"
                
                if [ $? -eq 0 ]; then
                    log_info "$repo updated successfully"
                else
                    log_error "Failed to update $repo"
                fi
            else
                log_warning "$repo is not a git repository. Skipping."
            fi
            
            cd ..
        else
            log_warning "Skipping $repo - directory not found"
        fi
    done
    
    log_info "Repository update complete"
}

setup_poetry_environments() {
    log_info "Setting up Poetry environments..."
    
    COMPONENTS=(
        "TeraSim"
        "TeraSim-Service"
        "TeraSim-NDE-NADE"
    )
    
    for component in "${COMPONENTS[@]}"; do
        if [ -d "$component" ]; then
            log_info "Configuring $component..."
            cd "$component"
            
            if [ -f "pyproject.toml" ]; then
                poetry config virtualenvs.in-project true
                
                # Check if poetry.lock exists and is in sync
                if [ -f "poetry.lock" ]; then
                    log_info "Checking poetry.lock for $component..."
                    if ! poetry lock --check &>/dev/null; then
                        log_warning "poetry.lock is out of sync for $component. Updating..."
                        poetry lock
                    fi
                else
                    log_warning "poetry.lock not found for $component. Creating..."
                    poetry lock
                fi
                
                # Force update lock file for TeraSim to ensure SUMO 1.23.1 is used
                if [ "$component" = "TeraSim" ]; then
                    log_info "Ensuring SUMO version 1.23.1 is locked for TeraSim..."
                    poetry lock
                fi
                
                log_info "Installing $component dependencies..."
                poetry install
                log_info "$component setup complete"
            else
                log_warning "pyproject.toml not found in $component"
            fi
            
            cd ..
        else
            log_warning "Skipping $component - directory not found"
        fi
    done
}

create_output_directories() {
    log_info "Creating necessary output directories..."
    mkdir -p outputs
    mkdir -p logs
    log_info "Output directories created"
}

display_menu() {
    echo
    echo "==================== Environment Setup Complete ===================="
    echo
    echo "Available commands:"
    echo "1. Start TeraSim service (port 8000)"
    echo "2. Run experiments"
    echo "3. Run debug experiments"
    echo "4. Check service status"
    echo "5. Stop service"
    echo "6. Exit"
    echo
}

run_service() {
    log_info "Starting TeraSim service..."
    
    if lsof -Pi :8000 -sTCP:LISTEN -t >/dev/null ; then
        log_warning "Port 8000 is already in use"
        read -p "Kill the process using port 8000? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            kill -9 $(lsof -t -i:8000)
            log_info "Port 8000 cleared"
        else
            return
        fi
    fi
    
    if [ -f "terasim_service_main.py" ]; then
        log_info "Starting service..."
        python terasim_service_main.py
    else
        log_error "terasim_service_main.py not found"
    fi
}

main() {
    log_info "Starting TeraSim-Deploy environment setup..."
    
    check_python
    check_poetry
    check_redis
    pull_latest_code
    download_repositories
    setup_poetry_environments
    create_output_directories
    
    log_info "Environment setup complete!"
    
    while true; do
        display_menu
        read -p "Select an option (1-6): " choice
        
        case $choice in
            1)
                run_service
                ;;
            2)
                if [ -f "run_experiments.py" ]; then
                    python run_experiments.py
                else
                    log_error "run_experiments.py not found"
                fi
                ;;
            3)
                if [ -f "run_experiments_debug.py" ]; then
                    python run_experiments_debug.py
                else
                    log_error "run_experiments_debug.py not found"
                fi
                ;;
            4)
                if lsof -Pi :8000 -sTCP:LISTEN -t >/dev/null ; then
                    log_info "TeraSim service is running (port 8000)"
                else
                    log_warning "TeraSim service is not running"
                fi
                ;;
            5)
                if lsof -Pi :8000 -sTCP:LISTEN -t >/dev/null ; then
                    kill -9 $(lsof -t -i:8000)
                    log_info "Service stopped"
                else
                    log_warning "Service not running"
                fi
                ;;
            6)
                log_info "Exiting"
                exit 0
                ;;
            *)
                log_error "Invalid choice"
                ;;
        esac
    done
}

main "$@"