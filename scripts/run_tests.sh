#!/bin/bash
# TeraSim Test Runner
# Comprehensive test script for the TeraSim monorepo

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper function for colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Default settings
RUN_ALL=true
RUN_CORE=false
RUN_ENVGEN=false
RUN_SERVICE=false
RUN_NDE_NADE=false
RUN_INTEGRATION=false
COVERAGE=true
VERBOSE=false
HTML_REPORT=false
SKIP_SLOW=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --core)
            RUN_CORE=true
            RUN_ALL=false
            shift
            ;;
        --envgen)
            RUN_ENVGEN=true
            RUN_ALL=false
            shift
            ;;
        --service)
            RUN_SERVICE=true
            RUN_ALL=false
            shift
            ;;
        --nde-nade)
            RUN_NDE_NADE=true
            RUN_ALL=false
            shift
            ;;
        --integration)
            RUN_INTEGRATION=true
            RUN_ALL=false
            shift
            ;;
        --no-coverage)
            COVERAGE=false
            shift
            ;;
        --html)
            HTML_REPORT=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --skip-slow)
            SKIP_SLOW=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --core           Run only core simulation tests"
            echo "  --envgen         Run only environment generation tests"
            echo "  --service        Run only service API tests"
            echo "  --nde-nade       Run only NDE-NADE component tests"
            echo "  --integration    Run only integration tests"
            echo "  --no-coverage    Skip coverage reporting"
            echo "  --html           Generate HTML coverage report"
            echo "  --verbose        Run tests in verbose mode"
            echo "  --skip-slow      Skip slow running tests"
            echo "  -h, --help       Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                    # Run all tests with coverage"
            echo "  $0 --core --verbose   # Run core tests with verbose output"
            echo "  $0 --integration      # Run only integration tests"
            echo "  $0 --skip-slow        # Skip slow tests"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Check if we're in the project root
if [ ! -f "pyproject.toml" ] || [ ! -d "tests" ]; then
    print_error "Please run this script from the TeraSim project root directory"
    exit 1
fi

# Check dependencies
print_status "Checking test dependencies..."

if ! command -v pytest &> /dev/null; then
    print_error "pytest not found. Please install it with: pip install pytest"
    exit 1
fi

if [ "$COVERAGE" = true ] && ! python -c "import pytest_cov" 2>/dev/null; then
    print_warning "pytest-cov not found. Coverage reporting will be disabled."
    COVERAGE=false
fi

# Check SUMO installation for integration tests
if [ "$RUN_ALL" = true ] || [ "$RUN_INTEGRATION" = true ]; then
    if [ -z "$SUMO_HOME" ]; then
        print_warning "SUMO_HOME not set. Integration tests requiring SUMO will be skipped."
    elif [ ! -d "$SUMO_HOME" ]; then
        print_warning "SUMO_HOME directory does not exist. Integration tests requiring SUMO will be skipped."
    fi
fi

print_success "Dependencies check completed"

# Build pytest command
PYTEST_ARGS=()

# Add verbosity
if [ "$VERBOSE" = true ]; then
    PYTEST_ARGS+=("-v")
else
    PYTEST_ARGS+=("-q")
fi

# Add coverage options
if [ "$COVERAGE" = true ]; then
    PYTEST_ARGS+=(
        "--cov=terasim" 
        "--cov=terasim_envgen" 
        "--cov=terasim_service" 
        "--cov=terasim_nde_nade"
        "--cov-report=term-missing"
    )
    
    if [ "$HTML_REPORT" = true ]; then
        PYTEST_ARGS+=("--cov-report=html")
    fi
fi

# Add test markers
if [ "$SKIP_SLOW" = true ]; then
    PYTEST_ARGS+=("-m" "not slow")
fi

# Determine which tests to run
TEST_PATHS=()

if [ "$RUN_ALL" = true ]; then
    TEST_PATHS+=("tests/")
else
    if [ "$RUN_CORE" = true ]; then
        TEST_PATHS+=("tests/test_core/")
    fi
    if [ "$RUN_ENVGEN" = true ]; then
        TEST_PATHS+=("tests/test_envgen/")
    fi
    if [ "$RUN_SERVICE" = true ]; then
        TEST_PATHS+=("tests/test_service/")
    fi
    if [ "$RUN_NDE_NADE" = true ]; then
        TEST_PATHS+=("tests/test_nde_nade/")
    fi
    if [ "$RUN_INTEGRATION" = true ]; then
        TEST_PATHS+=("tests/test_integration/")
    fi
fi

# Run tests
print_status "Starting test execution..."
print_status "Test paths: ${TEST_PATHS[*]}"
print_status "Pytest args: ${PYTEST_ARGS[*]}"

echo ""
echo "======================================"
echo "      TeraSim Test Suite"
echo "======================================"
echo ""

# Execute pytest
if pytest "${PYTEST_ARGS[@]}" "${TEST_PATHS[@]}"; then
    print_success "All tests passed!"
    
    if [ "$HTML_REPORT" = true ]; then
        print_status "HTML coverage report generated at: htmlcov/index.html"
    fi
    
    exit 0
else
    print_error "Some tests failed!"
    exit 1
fi