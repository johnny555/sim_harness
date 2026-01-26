#!/bin/bash
# Test runner script for the sim_harness package

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Parse command line arguments
RERUN_FAILED=false
SKIP_BUILD=false
TEST_TYPE="all"  # all, unit, integration

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -r, --rerun-failed    Rerun only tests that failed in the previous run"
    echo "  -s, --skip-build      Skip the build step"
    echo "  -t, --type TYPE       Run specific test type: all, unit, integration (default: all)"
    echo "  -h, --help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Run all tests"
    echo "  $0 -s                 # Skip build, run all tests"
    echo "  $0 -t unit            # Run only unit tests (Python + C++)"
    echo "  $0 -t integration     # Run only integration tests"
    echo "  $0 -r                 # Rerun failed tests from last run"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--rerun-failed)
            RERUN_FAILED=true
            shift
            ;;
        -s|--skip-build)
            SKIP_BUILD=true
            shift
            ;;
        -t|--type)
            TEST_TYPE="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            usage
            ;;
    esac
done

# Validate test type
if [[ "$TEST_TYPE" != "all" && "$TEST_TYPE" != "unit" && "$TEST_TYPE" != "integration" ]]; then
    echo -e "${RED}Invalid test type: $TEST_TYPE${NC}"
    echo "Valid types: all, unit, integration"
    exit 1
fi

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}sim_harness Test Runner${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# Get the script's directory and workspace root
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Change to workspace directory
cd "$WORKSPACE_ROOT"

# Auto-detect install layout (merged vs isolated)
INSTALL_LAYOUT_FILE="$SCRIPT_DIR/install/.colcon_install_layout"
COLCON_INSTALL_FLAG=""
if [ -f "$INSTALL_LAYOUT_FILE" ]; then
    LAYOUT=$(cat "$INSTALL_LAYOUT_FILE")
    if [ "$LAYOUT" = "merged" ]; then
        COLCON_INSTALL_FLAG="--merge-install"
        echo -e "${CYAN}Detected merged install layout${NC}"
    else
        echo -e "${CYAN}Detected isolated install layout${NC}"
    fi
else
    echo -e "${CYAN}No existing install layout detected, using default (isolated)${NC}"
fi

# Clean up any existing Gazebo/simulation processes
cleanup_simulation() {
    echo -e "${YELLOW}Cleaning up simulation processes...${NC}"
    # Kill any lingering Gazebo processes
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    pkill -9 ruby 2>/dev/null || true  # Gazebo uses ruby for some scripts
    # Give processes time to terminate
    sleep 1
}

cleanup_simulation

# Build the workspace (unless skipped)
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${YELLOW}Building sim_harness...${NC}"
    colcon build --packages-select sim_harness $COLCON_INSTALL_FLAG
    if [ $? -ne 0 ]; then
        echo -e "${RED}Build failed!${NC}"
        exit 1
    fi
    echo -e "${GREEN}Build successful!${NC}"
    echo ""
else
    echo -e "${YELLOW}Skipping build step...${NC}"
    echo ""
fi

# Source the workspace
source install/setup.bash

# Build ctest arguments based on test type
CTEST_ARGS=""
if [ "$TEST_TYPE" = "unit" ]; then
    echo -e "${CYAN}Running unit tests only...${NC}"
    CTEST_ARGS="-R 'test_.*_(python|cpp)' -E 'integration'"
elif [ "$TEST_TYPE" = "integration" ]; then
    echo -e "${CYAN}Running integration tests only...${NC}"
    CTEST_ARGS="-R 'integration'"
else
    echo -e "${CYAN}Running all tests...${NC}"
fi

if [ "$RERUN_FAILED" = true ]; then
    echo -e "${YELLOW}Rerunning only previously failed tests...${NC}"
    CTEST_ARGS="$CTEST_ARGS --rerun-failed --output-on-failure"
fi

# Run tests with live output
# Use --parallel-workers 1 for integration tests to avoid simulation conflicts
if [ "$TEST_TYPE" = "integration" ]; then
    echo -e "${YELLOW}Running integration tests sequentially (simulation tests)...${NC}"
    if [ -n "$CTEST_ARGS" ]; then
        colcon test --packages-select sim_harness $COLCON_INSTALL_FLAG --event-handlers console_direct+ --parallel-workers 1 --ctest-args $CTEST_ARGS
    else
        colcon test --packages-select sim_harness $COLCON_INSTALL_FLAG --event-handlers console_direct+ --parallel-workers 1
    fi
else
    if [ -n "$CTEST_ARGS" ]; then
        colcon test --packages-select sim_harness $COLCON_INSTALL_FLAG --event-handlers console_direct+ --ctest-args $CTEST_ARGS
    else
        colcon test --packages-select sim_harness $COLCON_INSTALL_FLAG --event-handlers console_direct+
    fi
fi

TEST_RESULT=$?
echo ""

# Clean up simulation processes after tests
cleanup_simulation

# Show test results summary
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Test Results Summary${NC}"
echo -e "${YELLOW}========================================${NC}"
colcon test-result --verbose 2>/dev/null || true
echo ""

# Exit with test result
if [ $TEST_RESULT -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}All tests passed!${NC}"
    echo -e "${GREEN}========================================${NC}"
    exit 0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Some tests failed!${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
