# Multi-Shape Rigidbody API Usage

## Overview

The physics system provides rigidbody simulation with iterative constraint solving, supporting multiple collision shapes per object.

**Important**: Only ONE Rigidbody component per object is allowed. Multiple shapes can be added to a single rigidbody.

## Editor Support

The Collision-Body component in the editor now supports all 4 shape types:
- **Sphere** - Defined by radius
- **Box** - Defined by half-size (width/2, height/2, depth/2)
- **Cylinder** - Defined by radius and half-height (Y-axis aligned)
- **Capsule** - Defined by radius and inner half-height (Y-axis aligned)

### Shape Parameters in Editor

- **Sphere**: Only radius parameter shown
- **Box**: Half Size (X, Y, Z) parameters
- **Cylinder**: Radius and Half Height parameters
- **Capsule**: Radius and Inner Half Height parameters

All shapes are visualized in the 3D viewport with cyan wireframes.

## Runtime API (For Code Components)

### Adding a Rigidbody Component

Rigidbodies are added via the editor or programmatically. Each object can have exactly one rigidbody.

```cpp
// Get the Rigidbody component (added in editor or during object creation)
auto* rigidbody = object->getComponent<Comp::CollBody>();
```

### Adding Multiple Shapes

A single rigidbody can have multiple collision shapes:

```cpp
// Clear default shape and add custom shapes
rigidbody->clearShapes();

// Add a capsule for the body
rigidbody->addCapsule(
    0.5f,  // radius
    1.0f,  // inner half height
    {0, 1.0f, 0}  // offset up by 1 unit
);

// Add a sphere for the head
rigidbody->addSphere(
    0.4f,  // radius
    {0, 2.5f, 0}  // offset up by 2.5 units
);

// Add a box for equipment
rigidbody->addBox(
    {0.3f, 0.2f, 0.3f},  // half extents
    {0.5f, 1.0f, 0}  // offset to the side
);
```

### Setting Physics Properties

```cpp
// Set mass (affects inertia)
rigidbody->setMass(70.0f);

// Set friction (0-1, higher = more friction)
rigidbody->setFriction(0.5f);

// Set bounce/restitution (0-1, higher = more bouncy)
rigidbody->setBounce(0.2f);

// Make kinematic (won't respond to physics)
rigidbody->setKinematic(false);
```

## Supported Shapes

### Sphere
```cpp
rigidbody->addSphere(radius, localOffset);
```

### Box
```cpp
rigidbody->addBox(halfExtents, localOffset);
// halfExtents: {width/2, height/2, depth/2}
```

### Cylinder (Y-axis aligned)
```cpp
rigidbody->addCylinder(radius, halfHeight, localOffset);
```

### Capsule (Y-axis aligned)
```cpp
rigidbody->addCapsule(radius, innerHalfHeight, localOffset);
// innerHalfHeight: height of cylindrical part (not including end caps)
```

## Shape Orientation

- All shapes inherit rotation from the parent object's transform
- Local offsets are applied in the object's local space
- The object's scale is applied to all shapes

## Single Rigidbody Constraint

**Each object can have only ONE Rigidbody component.**

If you try to add a second rigidbody, an error will be logged:
```
Error: Object X already has a Rigidbody component! Only one Rigidbody per object is allowed.
```

To create complex objects with multiple collision shapes, add them to the single rigidbody:
```cpp
// CORRECT: One rigidbody, multiple shapes
auto* rb = object->getComponent<Comp::CollBody>();
rb->addBox({1, 1, 1}, {0, 0, 0});
rb->addSphere(0.5f, {0, 2, 0});

// WRONG: Do not try to add multiple rigidbodies!
// object->addComponent<Comp::CollBody>(); // ERROR!
```

## Physics System Features

The rigidbody system includes:

1. **GJK/EPA Collision Detection** - Accurate narrow-phase for arbitrary convex shapes
2. **Iterative Constraint Solver** - 7 velocity + 4 position iterations
3. **Contact Caching** - Warm-starting for stable stacking
4. **Semi-Implicit Euler** - Stable velocity and position integration
5. **Friction & Restitution** - Realistic material properties
6. **Linear & Angular Dynamics** - Full 6-DOF simulation

## Integration with PhysicsScene

The PhysicsScene runs automatically in the game loop:

```cpp
// In Scene::update() - happens automatically
physicsScene.step(deltaTime);
```

This runs the complete physics pipeline:
1. Apply gravity
2. Integrate velocities
3. Detect contacts (broadphase + GJK/EPA)
4. Solve velocity constraints (7 iterations)
5. Integrate positions
6. Solve position constraints (4 iterations)

## Static Mesh Colliders

For static level geometry, use `CollMesh` components instead of rigidbodies. These provide efficient collision without physics simulation.

## Performance Notes

- Designed for N64 hardware constraints
- Fixed 256-contact cache (~75KB)
- ~200 bytes per rigidbody + shapes
- Suitable for 10-50 dynamic objects per scene

