# Collider and Rigidbody - Separate Components

## Overview

Following the pattern from `libdragon_tiny3d_test` and standard game engines (Unity, Unreal), the physics system now uses **two separate components**:

1. **Collider** - Defines collision shape and properties
2. **Rigidbody** - Enables physics simulation with mass and constraints

This matches the separation in libdragon_tiny3d_test:
- `physics_object_collision_data` → Collider
- `physics_object` → Rigidbody

## Component Separation

### Collider Component (ID 5)

**Purpose**: Defines what shape the object has for collision detection

**Properties**:
- Shape type (Sphere, Box, Cylinder, Capsule)
- Shape-specific parameters (radius, half-size, etc.)
- Offset from object origin
- **Friction** (0-1): Surface friction coefficient
- **Bounce/Restitution** (0-1): How much the object bounces
- **Is Trigger**: Pass-through collision (no physics response)
- **Collision Masks**: Which layers to read/write

**Can exist without Rigidbody**: Yes (for static collision geometry)

**Editor Display**: "Collider" with cube outline icon

### Rigidbody Component (ID 11)

**Purpose**: Enables physics simulation on the object

**Properties**:
- **Mass**: Object mass in kg (affects forces/impulses)
- **Use Gravity**: Whether gravity affects this object
- **Gravity Scale**: Multiplier for gravity effect
- **Is Kinematic**: Manually controlled (no physics forces)
- **Angular Damping**: How fast rotation slows down
- **Constraints** (6 flags):
  - Freeze Position X/Y/Z
  - Freeze Rotation X/Y/Z

**Requires Collider**: Yes (needs shape for collision detection)

**Editor Display**: "Rigidbody" with axis arrow icon

## Usage Patterns

### Static Collision Geometry
Objects that don't move but can be collided with:
```
✓ Collider component (defines shape)
✗ No Rigidbody component
```
Examples: Walls, floors, static obstacles

### Dynamic Physics Objects
Objects that are simulated by physics:
```
✓ Collider component (defines shape)
✓ Rigidbody component (enables simulation)
```
Examples: Crates, balls, dynamic platforms

### Kinematic Objects
Objects moved by code but interact with physics:
```
✓ Collider component (defines shape)
✓ Rigidbody component (with isKinematic = true)
```
Examples: Moving platforms, elevators, doors

### Trigger Volumes
Detects overlaps without physical response:
```
✓ Collider component (with isTrigger = true)
✗ Optional Rigidbody (if you want it to move)
```
Examples: Pickup zones, damage areas, checkpoints

## Editor Workflow

### Adding Physics to an Object

1. **Add Collider Component**:
   - Click "Add Component"
   - Select "Collider" (cube icon)
   - Choose shape type (Sphere, Box, etc.)
   - Set shape parameters
   - Set friction (0.5 default)
   - Set bounce (0.0 default)

2. **Add Rigidbody Component** (if object should simulate):
   - Click "Add Component"
   - Select "Rigidbody" (axis arrow icon)
   - Set mass (1.0 default)
   - Enable/disable gravity
   - Set constraints if needed

### Example: Creating a Ball

1. Add Collider:
   - Type: Sphere
   - Radius: 0.5
   - Friction: 0.3 (rolls easily)
   - Bounce: 0.8 (very bouncy)

2. Add Rigidbody:
   - Mass: 1.0 kg
   - Use Gravity: Yes
   - Gravity Scale: 1.0
   - Is Kinematic: No

### Example: Creating a Wall

1. Add Collider only:
   - Type: Box
   - Half Size: (0.5, 2.0, 5.0)
   - Friction: 0.5
   - Bounce: 0.0
   
No Rigidbody needed - it's static!

## Binary Format

### Collider Data (ID 5)
```
Offset | Size | Field
-------|------|-------
0x00   | 12   | halfExtend (vec3)
0x0C   | 12   | offset (vec3)
0x18   | 1    | flags (shape type + trigger)
0x19   | 4    | friction (float)
0x1D   | 4    | bounce (float)
0x21   | 1    | maskRead (uint8)
0x22   | 1    | maskWrite (uint8)
Total: 35 bytes
```

**Flags Format**:
- Bits 0-1: Shape type (00=Sphere, 01=Box, 10=Cylinder, 11=Capsule)
- Bit 2: Is trigger
- Bits 3-7: Reserved

### Rigidbody Data (ID 11)
```
Offset | Size | Field
-------|------|-------
0x00   | 4    | mass (float)
0x04   | 1    | gravityFlags (uint8, bit 0 = use gravity)
0x05   | 4    | gravityScale (float)
0x09   | 1    | isKinematic (uint8 bool)
0x0A   | 4    | angularDamping (float)
0x0E   | 2    | constraints (uint16)
Total: 16 bytes
```

**Constraints Format**:
- Bit 0: Freeze Position X
- Bit 1: Freeze Position Y
- Bit 2: Freeze Position Z
- Bit 3: Freeze Rotation X
- Bit 4: Freeze Rotation Y
- Bit 5: Freeze Rotation Z
- Bits 6-15: Reserved

## Runtime Behavior

### Initialization Order

1. **Collider initializes first** (ID 5 < ID 11):
   - Creates PhysicsBody
   - Sets up collision shape
   - Sets friction and bounce
   - Registers with PhysicsScene
   - **Default mass**: 1.0 kg

2. **Rigidbody initializes second**:
   - Finds Collider's PhysicsBody
   - Overrides mass with correct value
   - Sets kinematic flag
   - Sets gravity properties
   - Sets angular damping
   - Applies constraints

### Component Communication

The Rigidbody component finds the Collider via:
```cpp
CollBody* collider = obj.getComponent<CollBody>();
if (collider && collider->physicsBody) {
  // Apply rigidbody properties
  collider->physicsBody->setMass(data->mass);
  collider->physicsBody->setKinematic(data->isKinematic);
  // ...
}
```

### Physics Simulation

The PhysicsScene only looks at `CollBody` components for simulation:
- Iterates over all CollBody components
- Each CollBody has a PhysicsBody
- PhysicsBody contains both shape data (from Collider) and mass data (from Rigidbody)

## API Examples

### Code Component Access

```cpp
// Get components
auto* collider = obj->getComponent<Comp::CollBody>();
auto* rigidbody = obj->getComponent<Comp::Rigidbody>();

// Modify collider properties
collider->setFriction(0.8f);
collider->setBounce(0.5f);
collider->addSphere(1.0f, {0, 0, 0});

// Modify rigidbody properties (via physicsBody)
if (collider && collider->physicsBody) {
  collider->physicsBody->setMass(50.0f);
  collider->physicsBody->applyForce({0, 10, 0});
  collider->physicsBody->setKinematic(true);
}
```

## Migration from Old System

### Old System (Single Component)
Previously had one "Rigidbody" component that combined everything:
- Shape definition
- Mass
- Friction/bounce
- Constraints
- All mixed together

### New System (Two Components)
Now properly separated:
- **Collider**: Shape and collision properties
- **Rigidbody**: Mass and physics properties

### Why This Is Better

1. **Clarity**: Clear separation of concerns
2. **Flexibility**: Can have colliders without physics
3. **Standard**: Matches Unity, Unreal, libdragon_tiny3d_test
4. **Efficiency**: Static geometry doesn't need rigidbody data
5. **Extensibility**: Easy to add more component types

## Comparison with libdragon_tiny3d_test

### libdragon_tiny3d_test Structure

```c
// Collision data (shape and properties)
struct physics_object_collision_data {
  gjk_support_function gjk_support_function;
  bounding_box_calculator bounding_box_calculator;
  inertia_calculator inertia_calculator;
  union physics_object_collision_shape_data shape_data;
  Vector3 collider_world_center;
  physics_object_collision_shape_type shape_type;
  float bounce;
  float friction;
};

// Physics object (references collision data)
typedef struct physics_object {
  Vector3* position;
  Quaternion* rotation;
  Vector3 velocity;
  Vector3 angular_velocity;
  float _inv_mass;
  float _mass;
  float gravity_scalar;
  float angular_damping;
  struct physics_object_collision_data* collision; // POINTER TO COLLISION
  uint16_t constraints;
  bool has_gravity;
  bool is_kinematic;
  // ...
} physics_object;
```

### pyrite64 Equivalent

```cpp
// Collider component (ID 5)
struct CollBody {
  Physics::PhysicsBody* physicsBody;
  // Shape, friction, bounce stored in physicsBody
};

// Rigidbody component (ID 11)
struct Rigidbody {
  float mass;
  float gravityScale;
  float angularDamping;
  uint16_t constraints;
  bool isKinematic;
  // Applies to CollBody's PhysicsBody
};
```

The separation is conceptually identical!

## Conclusion

The new two-component system properly reflects the physics architecture:
- **Collider** = Collision shape and surface properties
- **Rigidbody** = Mass, motion, and constraints

This matches both industry standards and the reference implementation (libdragon_tiny3d_test).
