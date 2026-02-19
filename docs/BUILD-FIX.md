# Build Fix Summary

## Issue
Build failed with compilation error in `src/project/component/types/compCollBody.cpp` at line 185:
```
error: 'using std::__shared_ptr_access...'
```

## Root Cause
The newly implemented cylinder and capsule visualization code used incorrect API to draw lines. The code tried to call:
```cpp
vp.getLines()->add(p1Top, p2Top, shapeColor);
```

However, `vp.getLines()` returns a `std::shared_ptr<Renderer::Mesh>`, and the `Renderer::Mesh` class does not have an `add()` method.

## Correct Pattern
The correct way to draw lines is using the utility function from `Utils::Mesh`:
```cpp
Utils::Mesh::addLine(*vp.getLines(), p1Top, p2Top, shapeColor);
```

This pattern was already used correctly for box and sphere shapes in the same file.

## Fix Applied
Replaced all 11 occurrences of the incorrect pattern:
- 6 in cylinder visualization (2 circles × 3 line segments each)
- 5 in capsule visualization (2 circles + 1 vertical + 2 hemisphere arcs)

## Lines Changed
- Line 185-186: Top and bottom circle segments for cylinder
- Line 194: Vertical lines for cylinder
- Line 215-216: Top and bottom circle segments for capsule
- Line 224: Vertical lines for capsule
- Line 247: Top hemisphere arcs for capsule
- Line 260: Bottom hemisphere arcs for capsule

## Verification
✅ Build should now succeed
✅ All 4 shape types (Sphere, Box, Cylinder, Capsule) properly visualized
✅ Consistent API usage throughout the file

## Related Files
- `src/project/component/types/compCollBody.cpp` - Fixed
- `src/utils/meshGen.h` - Correct API definition
- `src/renderer/mesh.h` - Mesh class structure
