# Rigidbody Component - Naming and Structure

## Component Identity

### Editor Display
- **Name**: "Rigidbody"
- **Icon**: Axis arrow (ICON_MDI_AXIS_ARROW)
- **Purpose**: Physics-simulated rigidbody with constraint solver
- **ID**: 5

### Internal Structure (For Compatibility)
- **Engine Struct**: `P64::Comp::CollBody`
- **Editor Namespace**: `Project::Component::CollBody`
- **Source File**: `src/project/component/types/compCollBody.cpp`

## Why "Rigidbody" Not "Collision-Body"?

### Old System (Removed)
The old system was a simple collision body:
- Basic sphere/box collision detection
- No physics simulation
- No constraint solving
- Just collision response

### New System (Current)
The new system is a full physics rigidbody:
- ✅ GJK/EPA collision detection
- ✅ Iterative constraint solver (11 iterations)
- ✅ Semi-implicit Euler integration
- ✅ Linear and angular dynamics
- ✅ Friction and restitution
- ✅ Force and impulse accumulation
- ✅ Contact caching and warm-starting

**Result**: It's a rigidbody, not just a collision body!

## Naming Convention

### What Changed
| Aspect | Old | New | Reason |
|--------|-----|-----|--------|
| Editor Name | "Collision-Body" | "Rigidbody" | Reflects actual functionality |
| Icon | Cylinder | Axis Arrow | Represents 3D physics motion |
| Purpose | Collision detection | Full physics simulation | System upgrade |

### What Stayed the Same
| Aspect | Value | Reason |
|--------|-------|--------|
| Component ID | 5 | Binary compatibility |
| Engine struct | `CollBody` | Code compatibility |
| Editor namespace | `CollBody` | Minimal changes |
| Source file | `compCollBody.cpp` | Avoid breaking builds |

## User Impact

### In the Editor
Users will see:
```
Component List:
  ...
  [Axis Arrow Icon] Rigidbody
  ...
```

When adding a rigidbody to an object, it's clear they're adding a physics-simulated rigidbody, not just a collision volume.

### In Code
Engine code still uses:
```cpp
auto* rb = object->getComponent<Comp::CollBody>();
```

This maintains compatibility while the display name properly reflects the component's purpose.

## Documentation Alignment

All documentation now consistently refers to:
- **Rigidbody component** (not "collision body")
- **Physics simulation** (not just "collision")
- **Constraint solver** (the key feature)
- **One rigidbody per object** (the rule)

## Historical Context

1. **Original**: Collision-Body component with simple collision
2. **Migration**: Added new physics system (both systems coexisted)
3. **Replacement**: Removed old system, kept name "Collision-Body" (mistake)
4. **Fix**: Renamed to "Rigidbody" (this change)

The naming now accurately reflects what the component does: full rigidbody physics simulation.
