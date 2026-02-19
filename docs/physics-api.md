# Multi-Shape Collider API Usage

## Overview

The new physics system supports multiple collision shapes per object with iterative constraint solving.

## Basic Usage

### Converting Existing CollBody to New Physics

```cpp
#include "physics/collBodyBuilder.h"
#include "scene/components/collBody.h"

// Get existing CollBody component
auto* collBody = object->getComponent<Comp::CollBody>();

// Enable new physics system
Physics::CollBodyShapeBuilder::enableNewPhysics(collBody);

// Clear default shape and add custom shapes
Physics::CollBodyShapeBuilder::clearShapes(collBody);
```

### Adding Multiple Shapes

```cpp
// Add a capsule for the body
Physics::CollBodyShapeBuilder::addCapsule(
    collBody, 
    0.5f,  // radius
    1.0f,  // inner half height
    {0, 1.0f, 0}  // offset up by 1 unit
);

// Add a sphere for the head
Physics::CollBodyShapeBuilder::addSphere(
    collBody,
    0.4f,  // radius
    {0, 2.5f, 0}  // offset up by 2.5 units
);

// Add a box for equipment
Physics::CollBodyShapeBuilder::addBox(
    collBody,
    {0.3f, 0.2f, 0.3f},  // half extents
    {0.5f, 1.0f, 0}  // offset to the side
);
```

### Setting Physics Properties

```cpp
// Set mass (affects inertia)
Physics::CollBodyShapeBuilder::setMass(collBody, 70.0f);

// Set friction (0-1, higher = more friction)
Physics::CollBodyShapeBuilder::setFriction(collBody, 0.5f);

// Set bounce/restitution (0-1, higher = more bouncy)
Physics::CollBodyShapeBuilder::setBounce(collBody, 0.2f);

// Make kinematic (won't respond to physics)
Physics::CollBodyShapeBuilder::setKinematic(collBody, false);
```

## Supported Shapes

### Sphere
```cpp
Physics::CollBodyShapeBuilder::addSphere(body, radius, localOffset);
```

### Box
```cpp
Physics::CollBodyShapeBuilder::addBox(body, halfExtents, localOffset);
// halfExtents: {width/2, height/2, depth/2}
```

### Cylinder (Y-axis aligned)
```cpp
Physics::CollBodyShapeBuilder::addCylinder(body, radius, halfHeight, localOffset);
```

### Capsule (Y-axis aligned)
```cpp
Physics::CollBodyShapeBuilder::addCapsule(body, radius, innerHalfHeight, localOffset);
// innerHalfHeight: height of cylindrical part (not including end caps)
```

## Shape Orientation

- All shapes inherit rotation from the parent object's transform
- Local offsets are applied in the object's local space
- The object's scale is applied to all shapes

## Backward Compatibility

By default, CollBody uses the legacy collision system. The new physics must be explicitly enabled:

```cpp
Physics::CollBodyShapeBuilder::enableNewPhysics(collBody);
```

Legacy single-shape colliders will continue to work without modification.

## Integration with PhysicsScene

The PhysicsScene must be integrated into the main game loop:

```cpp
// In Scene update:
physicsScene.step(deltaTime);
```

This runs the complete physics pipeline:
1. Apply gravity
2. Integrate velocities
3. Detect contacts (broadphase + GJK/EPA)
4. Solve velocity constraints (7 iterations)
5. Integrate positions
6. Solve position constraints (4 iterations)

## Contact Caching

The system automatically caches contact constraints between frames for warm-starting, improving solver convergence and stability.
