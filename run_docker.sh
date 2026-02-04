#!/bin/bash
# =============================================================================
# Docker Navigation Test Runner
# =============================================================================
# This script builds and runs the ROS2/Gazebo navigation tests in Docker
# to avoid WSL2 DDS issues.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║       ROS2 Navigation Planner Comparison (Docker)             ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed.${NC}"
    echo "Install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# Allow X11 forwarding
echo -e "${YELLOW}Setting up X11 display...${NC}"
xhost +local:docker 2>/dev/null || echo "Note: xhost not available (GUI may not work)"

# Function to run with a specific planner
run_planner() {
    local planner=$1
    echo -e "${GREEN}"
    echo "============================================"
    echo "  Running with ${planner} planner"
    echo "============================================"
    echo -e "${NC}"
    
    if [ "$planner" == "smac" ]; then
        docker-compose up ros2_nav
    else
        docker-compose --profile navfn up ros2_nav_navfn
    fi
}

# Parse arguments
case "${1:-smac}" in
    smac|SmacPlanner2D)
        echo -e "${BLUE}Selected: SmacPlanner2D${NC}"
        echo ""
        echo "Building Docker image (first time may take 5-10 minutes)..."
        docker-compose build ros2_nav
        run_planner "smac"
        ;;
    navfn|NavFn)
        echo -e "${BLUE}Selected: NavFn${NC}"
        echo ""
        echo "Building Docker image..."
        docker-compose build ros2_nav_navfn
        run_planner "navfn"
        ;;
    compare)
        echo -e "${BLUE}Running comparison test...${NC}"
        echo ""
        echo "Step 1: Building Docker image..."
        docker-compose build
        
        echo ""
        echo -e "${GREEN}Step 2: Running SmacPlanner2D test...${NC}"
        timeout 300 docker-compose up ros2_nav || true
        
        echo ""
        echo -e "${GREEN}Step 3: Running NavFn test...${NC}"
        timeout 300 docker-compose --profile navfn up ros2_nav_navfn || true
        
        echo ""
        echo -e "${GREEN}Comparison complete! Check /tmp/nav_metrics/ for results.${NC}"
        ;;
    shell)
        echo -e "${BLUE}Opening interactive shell...${NC}"
        docker-compose build ros2_nav
        docker-compose run --rm ros2_nav bash
        ;;
    clean)
        echo -e "${YELLOW}Cleaning up Docker resources...${NC}"
        docker-compose down -v --rmi local
        echo -e "${GREEN}Cleanup complete.${NC}"
        ;;
    *)
        echo "Usage: $0 [smac|navfn|compare|shell|clean]"
        echo ""
        echo "Options:"
        echo "  smac     - Run with SmacPlanner2D (default)"
        echo "  navfn    - Run with NavFn planner"
        echo "  compare  - Run both planners and compare"
        echo "  shell    - Open interactive shell in container"
        echo "  clean    - Remove Docker images and containers"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}Done!${NC}"

