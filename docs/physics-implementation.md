# Physics System Implementation Summary

## Overview

This implementation ports the iterative constraint solver and collision system from the libdragon_tiny3d_test repository into pyrite64's N64 runtime. The system supports multiple collision shapes per object with stable, frame-coherent physics simulation.

## Architecture

### Core Components

1. **PhysicsScene** (`n64/engine/include/physics/physicsScene.h`)
   - Manages all physics bodies and constraint solving
   - Implements 7-iteration velocity solver + 4-iteration position solver
   - Contact constraint caching with hash map for warm-starting
   - ~350 lines of core solver logic

2. **PhysicsBody** (`n64/engine/include/physics/physicsBody.h`)
   - Represents a dynamic physics object
   - Supports velocity, angular velocity, mass, friction, bounce
   - Can hold multiple collision shapes

3. **ColliderShape** (`n64/engine/include/physics/shapes.h`)
   - Shape union supporting: Sphere, Box, Cylinder, Capsule
   - Each shape has local offset from body center
   - Rotation inherited from parent object transform

4. **ContactConstraint** (`n64/engine/include/physics/contact.h`)
   - Cached contact data between object pairs
   - Up to 4 contact points per pair
   - Stores accumulated impulses for warm-starting

### Collision Detection Pipeline

1. **Broadphase**: Simple O(n²) pair checking (could optimize with existing BVH)
2. **Narrowphase**: GJK algorithm for overlap detection
3. **EPA**: Expanding Polytope Algorithm for penetration depth/normal
4. **Contact Generation**: Creates/updates cached constraint data

### Constraint Solver Pipeline

The physics step (`PhysicsScene::step`) follows this sequence:

```
1. Apply gravity and integrate velocities (semi-implicit Euler)
2. Detect all contacts (broadphase → GJK → EPA)
3. Pre-solve: Calculate effective masses for all contact points
4. Warm-start: Apply cached impulses from previous frame
5. Solve velocity constraints (7 iterations)
   - Normal impulses (non-penetration)
   - Friction impulses (Coulomb friction cone)
6. Integrate positions from velocities
7. Solve position constraints (4 iterations with Baumgarte)
8. Update sleep states (placeholder)
```

## File Structure

### Headers
```
n64/engine/include/physics/
├── shapes.h           - Shape types and support functions
├── physicsBody.h      - Physics body with dynamics
├── contact.h          - Contact constraints and points
├── gjk.h              - GJK overlap detection
├── epa.h              - EPA penetration depth
├── physicsScene.h     - Main physics scene
└── collBodyBuilder.h  - Runtime API for shape management
```

### Implementation
```
n64/engine/src/physics/
├── shapes.cpp         - Shape AABB and support functions
├── physicsBody.cpp    - Body integration and impulses
├── gjk.cpp            - GJK algorithm (~220 lines)
├── epa.cpp            - EPA algorithm (~180 lines)
├── physicsScene.cpp   - Complete solver pipeline (~580 lines)
└── collBodyBuilder.cpp - Shape builder API
```

## Integration Points

1. **Scene class** (`n64/engine/include/scene/scene.h`)
   - Added `Physics::PhysicsScene physicsScene` member
   - Added `getPhysics()` accessor
   - Physics step called in `Scene::update()` after legacy collision

2. **CollBody component** (`n64/engine/include/scene/components/collBody.h`)
   - Added `Physics::PhysicsBody* physicsBody` member
   - Added `bool useNewPhysics` flag
   - Maintains backward compatibility with legacy BCS system

3. **Makefile** (`n64/engine/Makefile`)
   - Added `src/physics/*.cpp` to build

## Backward Compatibility

**BREAKING CHANGE**: The new physics system **completely replaces** the old collision system for dynamic objects. There is **no backward compatibility**.

### What Changed

1. **CollBody is now Rigidbody-only** - Always uses new physics, no opt-in flag
2. **One rigidbody per object** - Strictly enforced with error logging
3. **Direct API** - Shape methods on component (no builder class)
4. **Static geometry separate** - CollMesh still uses legacy system (intentional)

### Migration Required

**Old code (will NOT work)**:
```cpp
Physics::CollBodyShapeBuilder::enableNewPhysics(collBody);
```

**New code**:
```cpp
auto* rb = object->getComponent<Comp::CollBody>();
rb->addBox(halfSize, offset);  // Direct methods
```

See `docs/PHYSICS-REPLACEMENT.md` for complete migration guide.

## Key Algorithms Ported

### GJK (Gilbert-Johnson-Keerthi)
- Iterative simplex-based overlap detection
- Works with arbitrary convex shapes via support functions
- Typically converges in <10 iterations

### EPA (Expanding Polytope Algorithm)
- Finds penetration depth and contact normal
- Starts from GJK's final simplex
- Iteratively expands polytope toward Minkowski difference boundary

### Sequential Impulse Solver
- Gauss-Seidel iteration over contact constraints
- Separate normal and friction impulses
- Accumulated impulses clamped to physical limits
- Warm-starting from previous frame's solution

## Shape Support Functions

All shapes implement support functions for GJK:

```cpp
// Sphere: furthest point on surface
result = center + direction * radius

// Box: furthest corner
result = center + sign(direction) * halfExtents

// Cylinder (Y-axis aligned): cap + radial
y = sign(direction.y) * halfHeight
radial = normalize(direction.xz) * radius
result = center + (radial, y, radial)

// Capsule (Y-axis aligned): sphere + cylinder
y = sign(direction.y) * innerHalfHeight
result = center + direction * radius + (0, y, 0)
```

## Contact Caching Strategy

Contacts are cached between frames using:
- **Key**: ContactPairId (combination of two object IDs)
- **Value**: ContactConstraint with up to 4 points
- **Validation**: Points marked active/inactive each frame
- **Benefit**: Warm-starting accelerates convergence

## Performance Characteristics

### Complexity
- Broadphase: O(n²) - could optimize with spatial partitioning
- Narrowphase per pair: O(1) average (GJK iterations)
- Constraint solver: O(c * i) where c=contacts, i=iterations

### Memory
- Fixed contact cache: 256 constraints max
- Each constraint: ~300 bytes (4 points × ~75 bytes)
- Total cache: ~75KB

## Testing & Validation

### Code Review
- Run `code_review` tool before finalizing
- Address any identified issues

### CodeQL Security
- Run `codeql_checker` for vulnerability scanning
- Fix any critical issues found

### Build Verification
Requires N64 toolchain (libdragon):
```bash
cd n64/engine
make clean
make
```

### Runtime Testing
1. Enable new physics on a test object
2. Add multiple shapes
3. Verify:
   - Objects collide correctly
   - Solver converges (no jitter)
   - Friction and bounce work
   - Performance is acceptable

## Limitations & Future Work

### Current Limitations
1. Simplified inertia tensor (identity matrix approximation)
2. No sleep/wake system (placeholder only)
3. No constraint joints (hinges, springs, etc.)
4. No continuous collision detection (CCD)
5. No editor UI for multi-shape editing

### Future Enhancements
1. Proper inertia tensor calculation per shape
2. Island-based sleep/wake for optimization
3. Constraint joints system
4. Swept collision for fast-moving objects
5. Editor visual tools for shape composition
6. Broadphase acceleration (sweep-and-prune or BVH)

## References

Source material from libdragon_tiny3d_test:
- `src/collision/collision_scene.c` - Main solver pipeline
- `src/collision/gjk.c` - GJK implementation
- `src/collision/epa.c` - EPA implementation
- `src/collision/shapes.c` - Shape support functions
- `src/collision/collide.c` - Contact generation

## Credits

- **Original Implementation**: libdragon_tiny3d_test repository
- **Port & Adaptation**: This implementation for pyrite64
- **Algorithm Sources**: GJK (1988), EPA (van den Bergen), Sequential Impulse (Erin Catto)
