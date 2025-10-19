# Quick Start Guide

Get up and running with the Human Perception System in 5 minutes!

## Prerequisites

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git doxygen graphviz
```

## Build and Run

```bash
# 1. Navigate to project directory
cd /path/to/phase0

# 2. Configure project
cmake -S ./ -B build/

# 3. Build project
cmake --build build/

# 4. Run demo application
./build/app/shell-app

# 5. Run unit tests
cd build && ctest
```

## Expected Output

### Demo Application Output

You should see:
```
================================================
   Acme Robotics - Human Perception System     
   Phase 0: Design and Architecture Demo       
================================================

[Frame 1] Detected 2 humans
  Detection at: (-0.755556, 0, 5.22222) m
  Active tracks: 0
...
```

### Test Output

You should see:
```
97% tests passed, 1 tests failed out of 35
Total Test time (real) = 0.13 sec
```

The one failing test is intentional (legacy test).

## Next Steps

### View Documentation

```bash
# Generate and view Doxygen docs
cmake --build build/ --target docs
open docs/html/index.html
```

### View UML Diagrams

```bash
# Install PlantUML
sudo apt-get install plantuml

# Generate diagrams
plantuml docs/uml/*.puml

# View diagrams
open docs/uml/*.png
```

### Run Static Analysis

```bash
# Install cppcheck
sudo apt-get install cppcheck

# Run analysis
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem \
  --inline-suppr --quiet include/ libs/ app/ test/
```

### Generate Code Coverage

```bash
# Install coverage tools
sudo apt-get install gcovr lcov

# Configure with coverage
cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/

# Build and run tests with coverage
cmake --build build/ --clean-first --target all test_coverage

# View coverage report
open build/test_coverage/index.html
```

## Troubleshooting

### Problem: CMake can't find GoogleTest

**Solution**: CMake will automatically download GoogleTest. Ensure you have internet connection.

### Problem: Compiler doesn't support C++17

**Solution**: Update your compiler:
```bash
sudo apt-get install gcc-9 g++-9
export CXX=g++-9
```

### Problem: Build fails with OpenCV errors

**Solution**: OpenCV is optional for Phase 0. If installed, CMake will use it, but it's not required.

## Project Structure

```
phase0/
├── include/perception/     # Header files with Doxygen docs
├── libs/perception/        # Implementation files
├── app/                    # Main application
├── test/                   # Unit tests
├── docs/                   # Documentation and UML diagrams
├── build/                  # Build directory (generated)
└── README.md              # Full documentation
```

## Key Files

- **README.md**: Comprehensive project documentation
- **PHASE0_COMPLETION_REPORT.md**: Detailed completion report
- **AIP_Document.md**: Agile process documentation
- **docs/uml/**: UML diagrams (class, sequence, activity)

## Support

For issues or questions:
1. Check README.md troubleshooting section
2. Review UML diagrams for architecture understanding
3. Read Doxygen documentation for API details
4. Contact team members (see README.md for contact info)

## Phase 1 Preview

Phase 1 will add:
- Real YOLO model integration
- Actual Kalman filter mathematics
- Video/camera processing
- Visualization
- 90%+ code coverage

Stay tuned!

---

**Happy Coding!** 🚀

