# Developer Documentation

Development in sunnypilot happens in small incremental steps. The development process looks like:

1. Pick an issue to work on from our [GitHub issues](https://github.com/sunnyhaibin/sunnypilot/issues)
2. Implement the change
3. Ensure all new features have proper validation and safety checks
4. Document the change in the commit message
5. Submit a pull request on GitHub with comprehensive testing results
6. Work with reviewers to get the code merged

sunnypilot development is collaborative. All code changes go through code review.

## Enhanced System Components

When developing for sunnypilot, be aware of the following enhanced components:

### Enhanced Validation System
- All perception outputs must go through the enhanced validation pipeline
- Implement proper confidence scoring with situation awareness
- Follow the validation metrics publisher/consumer pattern
- Include temporal consistency checks

### ARM NEON Optimizations
- Use ARM NEON optimizations for performance-critical code paths
- Leverage specialized ARM instructions for mathematical operations
- Implement efficient memory access patterns for ARM processors
- Profile code on ARM hardware to verify improvements

### Advanced UI System
- Use the Raylib UI system for efficient rendering
- Implement layered UI architecture with proper z-ordering
- Follow resource-efficient rendering practices
- Include system status and validation metrics in UI

### Predictive Planning
- Integrate behavior prediction for improved planning
- Implement scenario-aware planning decisions
- Use Kalman filters for object tracking
- Include predictive safety checks

### Safety Supervisor
- All safety-critical code must go through safety supervisor validation
- Implement redundant safety checks
- Include emergency response protocols
- Follow safety-first design principles

## Development Best Practices

### Performance Optimization
- Profile code regularly with ARM-specific tools
- Use memory pooling to reduce allocation overhead
- Implement efficient data structures for ARM processors
- Validate performance against hardware constraints

### Safety-First Development
- All changes must maintain safety requirements
- Implement proper validation for new features
- Follow defensive programming practices
- Test extensively under various conditions

### Code Quality
- Follow existing code style and conventions
- Write comprehensive unit tests
- Include performance benchmarks
- Document performance characteristics