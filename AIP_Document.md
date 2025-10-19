# Agile Iterative Process (AIP) Document

**Project**: Human Perception System (HPS)  
**Team Members**: [Student 1 Name], [Student 2 Name]  
**Course**: ENPM700 - Software Development for Robotics  
**Institution**: University of Maryland  
**Semester**: Fall 2025  

---

## Product Backlog

### Phase 0: Design and Architecture (Week 1)

| ID | User Story | Priority | Effort | Status |
|----|-----------|----------|--------|--------|
| 1.1 | As a developer, I want clear interfaces for detection and tracking so that I can swap algorithms easily | High | 5 | ✅ Done |
| 1.2 | As a developer, I want comprehensive class diagrams so that I understand the system architecture | High | 3 | ✅ Done |
| 1.3 | As a developer, I want stub implementations so that I can build and test the system structure | High | 8 | ✅ Done |
| 1.4 | As a developer, I want UML diagrams (class, sequence, activity) so that I can visualize the workflow | High | 5 | ✅ Done |
| 1.5 | As a developer, I want unit test structure so that I can verify each component | High | 8 | ✅ Done |
| 1.6 | As a developer, I want Doxygen comments in headers so that I have API documentation | High | 5 | ✅ Done |
| 1.7 | As a developer, I want a comprehensive README so that others can understand and build the project | Medium | 3 | ✅ Done |
| 1.8 | As a developer, I want CMake build system configured so that the project builds cleanly | High | 3 | ✅ Done |

**Total Effort**: 40 story points  
**Sprint Goal**: Complete system architecture design with buildable stub implementation

### Phase 1: Core Implementation (Week 2-3)

| ID | User Story | Priority | Effort | Status |
|----|-----------|----------|--------|--------|
| 2.1 | As a user, I want YOLO to detect humans in images so that I can track people | High | 13 | 🚧 Pending |
| 2.2 | As a user, I want Kalman filter tracking so that I can follow humans across frames | High | 13 | 🚧 Pending |
| 2.3 | As a user, I want coordinate transformation so that positions are in robot frame | High | 8 | 🚧 Pending |
| 2.4 | As a user, I want video processing so that I can analyze recorded footage | Medium | 5 | 🚧 Pending |
| 2.5 | As a user, I want live camera processing so that I can run real-time detection | Medium | 5 | 🚧 Pending |
| 2.6 | As a developer, I want integration tests so that I verify the full pipeline works | High | 8 | 🚧 Pending |
| 2.7 | As a developer, I want 90%+ code coverage so that I ensure quality | High | 8 | 🚧 Pending |
| 2.8 | As a developer, I want GitHub CI/CD pipeline so that tests run automatically | Medium | 5 | 🚧 Pending |
| 2.9 | As a user, I want visualization of detections and tracks so that I can debug visually | Low | 5 | 🚧 Pending |

**Total Effort**: 70 story points

### Phase 2: Testing and Refinement (Week 4)

| ID | User Story | Priority | Effort | Status |
|----|-----------|----------|--------|--------|
| 3.1 | As a user, I want accurate depth estimation so that 3D positions are reliable | High | 8 | 🚧 Pending |
| 3.2 | As a developer, I want performance benchmarks so that I know FPS capabilities | Medium | 5 | 🚧 Pending |
| 3.3 | As a user, I want robust tracking so that tracks don't get lost easily | High | 8 | 🚧 Pending |
| 3.4 | As a developer, I want static analysis (cppcheck) passing so that code quality is high | Medium | 3 | 🚧 Pending |
| 3.5 | As a developer, I want CodeCov badge showing 90%+ so that others see test quality | Medium | 3 | 🚧 Pending |
| 3.6 | As a user, I want sample videos/demos so that I can see the system working | Low | 5 | 🚧 Pending |
| 3.7 | As a developer, I want performance optimization so that system runs at 30 FPS | Medium | 8 | 🚧 Pending |

**Total Effort**: 40 story points

---

## Sprint Planning

### Sprint 0 (Phase 0): Design and Architecture

**Duration**: Week 1  
**Sprint Goal**: Design complete system architecture with buildable stub implementation  

#### Sprint Backlog

| Task | Assigned To | Estimated Hours | Actual Hours | Status |
|------|-------------|----------------|--------------|--------|
| Design class hierarchy and interfaces | Team | 4 | 4 | ✅ Done |
| Create header files with Doxygen | Driver | 6 | 6 | ✅ Done |
| Implement stub classes (compile only) | Driver | 8 | 8 | ✅ Done |
| Create unit test structure | Navigator | 6 | 6 | ✅ Done |
| Design UML diagrams (class, sequence, activity) | Team | 4 | 4 | ✅ Done |
| Setup CMake build system | Navigator | 3 | 3 | ✅ Done |
| Write comprehensive README | Team | 3 | 3 | ✅ Done |
| Review and pair programming | Team | 6 | 6 | ✅ Done |

**Total Hours**: 40 hours (20 hours per person)

#### Sprint Review

**What went well**:
- Clear class hierarchy with well-defined interfaces
- Comprehensive Doxygen documentation in all headers
- Unit test structure covers all major components
- UML diagrams effectively communicate design
- CMake builds cleanly without errors
- Used design patterns (Strategy, Facade, Factory, RAII)

**What could be improved**:
- Could add more edge case tests
- Performance benchmarks not yet implemented
- Need real YOLO model integration in Phase 1

**Velocity**: 40 story points completed

---

## Risk Management

### Identified Risks

| Risk | Probability | Impact | Mitigation Strategy | Status |
|------|------------|--------|---------------------|--------|
| YOLO model integration complexity | Medium | High | Use pre-trained models, OpenCV DNN module | ⚠️ Monitor |
| OpenCV dependency issues | Low | Medium | Provide installation instructions, use stub for Phase 0 | ✅ Mitigated |
| Real-time performance not achieved | Medium | High | Profile code, optimize hotspots, use threading | 🚧 To Address |
| Depth estimation inaccurate | Medium | Medium | Use average human height, allow calibration | 🚧 To Address |
| Data association fails with occlusion | Low | Medium | Hungarian algorithm, track re-ID (Phase 2 optional) | 🚧 To Address |
| Code coverage below 90% | Low | Low | Write comprehensive tests during development | ✅ Mitigated |

---

## Meeting Notes

### Sprint 0 Planning Meeting

**Date**: [Date]  
**Attendees**: [Student 1], [Student 2]  
**Duration**: 2 hours

**Decisions**:
1. Use YOLOv8 for detection (AGPL-3.0 license OK for academic use)
2. Kalman Filter for tracking (simple, proven algorithm)
3. Stub implementation for Phase 0 (mock detections)
4. GoogleTest for unit testing
5. PlantUML for UML diagrams
6. Follow Google C++ Style Guide

**Action Items**:
- [x] Student 1: Design interfaces and create headers
- [x] Student 2: Setup CMake and create unit tests
- [x] Both: Review code and pair program for implementations
- [x] Both: Create UML diagrams and documentation

### Daily Standups (Phase 0)

#### Day 1
- **Student 1**: Created IDetector, ITracker, ICoordinateTransform interfaces
- **Student 2**: Setup CMake structure, added GoogleTest
- **Blockers**: None

#### Day 2
- **Student 1**: Implemented Detection, BoundingBox, YOLODetector headers
- **Student 2**: Created unit test structure, started test cases
- **Blockers**: None

#### Day 3
- **Student 1**: Implemented Track, KalmanFilter, KalmanTracker headers
- **Student 2**: Continued unit tests, added stub implementations
- **Blockers**: None

#### Day 4
- **Student 1**: Implemented Core module (Pipeline, Camera, Transformer)
- **Student 2**: Created main application, updated CMake
- **Blockers**: None

#### Day 5
- **Student 1**: Created UML diagrams (class, sequence, activity)
- **Student 2**: Wrote comprehensive README, documentation
- **Blockers**: None

---

## Pair Programming Log

| Date | Driver | Navigator | Task | Duration | Notes |
|------|--------|-----------|------|----------|-------|
| [Date] | Student 1 | Student 2 | Design interfaces | 2 hours | Discussed Strategy pattern |
| [Date] | Student 2 | Student 1 | Setup CMake | 1.5 hours | Added perception library |
| [Date] | Student 1 | Student 2 | Implement stub classes | 3 hours | Focused on Detection module |
| [Date] | Student 2 | Student 1 | Write unit tests | 3 hours | Comprehensive test coverage |
| [Date] | Student 1 | Student 2 | Create UML diagrams | 2 hours | Used PlantUML |
| [Date] | Student 2 | Student 1 | Write documentation | 2 hours | Comprehensive README |

**Total Pair Programming**: ~14 hours

---

## Code Review Checklist

### Phase 0 Review

- [x] All classes have proper Doxygen comments
- [x] Interfaces follow Strategy pattern
- [x] SOLID principles applied
- [x] Google C++ Style Guide followed
- [x] Unit test structure in place
- [x] CMake builds without errors
- [x] Header guards and include guards correct
- [x] No memory leaks (using smart pointers)
- [x] Const correctness maintained
- [x] UML diagrams match implementation

---

## Burndown Chart

### Sprint 0 (Phase 0)

| Day | Story Points Remaining | Ideal Burndown |
|-----|----------------------|----------------|
| 0   | 40                   | 40             |
| 1   | 32                   | 32             |
| 2   | 24                   | 24             |
| 3   | 16                   | 16             |
| 4   | 8                    | 8              |
| 5   | 0                    | 0              |

**Result**: Completed on schedule ✅

---

## Retrospective

### What Went Well 🎉
1. Clear design with well-defined interfaces
2. Strong understanding of design patterns
3. Comprehensive documentation from the start
4. Good pair programming collaboration
5. All stub implementations compile successfully
6. Unit test structure covers all modules
7. UML diagrams clearly communicate architecture

### What Didn't Go Well 😞
1. Initial confusion about YOLO licensing (resolved)
2. Took time to decide on coordinate frame conventions
3. CMake configuration needed a few iterations

### Action Items for Next Sprint 🚀
1. Start YOLO integration early (most complex task)
2. Setup CI/CD pipeline at beginning of sprint
3. Create test data repository with sample images/videos
4. Research Kalman filter matrix libraries (consider Eigen)
5. Profile code early to identify bottlenecks

### Team Feedback
- **Student 1**: "Great collaboration, clear division of work. Need to improve time estimation."
- **Student 2**: "Learned a lot about design patterns. Should start testing earlier next sprint."

---

## Definition of Done

### Phase 0 (Design)
- [x] All class headers created with complete signatures
- [x] Doxygen comments on all public APIs
- [x] Stub implementations compile successfully
- [x] Unit test structure in place (tests may use mocks)
- [x] UML diagrams created (class, sequence, activity)
- [x] CMake builds without errors or warnings
- [x] README with comprehensive documentation
- [x] Code follows Google C++ Style Guide
- [x] All team members reviewed and approved design

### Phase 1 (Implementation) - Future
- [ ] YOLO model loads and runs inference
- [ ] Kalman filter performs actual state estimation
- [ ] Coordinate transformation produces accurate results
- [ ] Video/camera processing functional
- [ ] Unit tests pass with 90%+ coverage
- [ ] Integration tests pass
- [ ] CI/CD pipeline green
- [ ] Code reviewed by both team members

---

## Tools and Technologies

| Tool | Purpose | Version |
|------|---------|---------|
| C++ | Implementation | C++17 |
| CMake | Build system | 3.14+ |
| GoogleTest | Unit testing | 1.14.0 |
| OpenCV | Computer vision | 4.0+ (Phase 1) |
| YOLOv8 | Human detection | Latest (Phase 1) |
| Doxygen | Documentation | Latest |
| PlantUML | UML diagrams | Latest |
| Git | Version control | Latest |
| GitHub | Repository hosting | - |
| cppcheck | Static analysis | Latest |
| gcovr/lcov | Code coverage | Latest |

---

## Next Steps (Phase 1)

1. **Week 2 Sprint Planning**
   - Prioritize YOLO integration
   - Setup test data repository
   - Configure CI/CD pipeline
   - Begin Kalman filter implementation

2. **Technical Tasks**
   - Download and test YOLOv8 ONNX model
   - Integrate OpenCV DNN module
   - Implement actual Kalman filter math
   - Add video/camera capture
   - Write integration tests

3. **Documentation**
   - Update README with build instructions for OpenCV
   - Add algorithm details and math derivations
   - Create user guide with examples
   - Add troubleshooting section

---

**Document Version**: 1.0  
**Last Updated**: [Current Date]  
**Status**: Phase 0 Complete ✅

