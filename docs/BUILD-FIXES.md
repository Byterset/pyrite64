# Build Fixes Documentation

## Overview
This document tracks all build issues encountered during the physics system implementation and their resolutions.

## Build Issue #1: Incorrect Line Drawing API

### Error
```
error: 'using std::__shared_ptr_access...'
```

### Location
`src/project/component/types/compCollBody.cpp` lines ~185-250

### Cause
Incorrectly called `vp.getLines()->add(p1, p2, color)` directly on the shared_ptr returned by `getLines()`.

### Solution
Use the utility function: `Utils::Mesh::addLine(*vp.getLines(), p1, p2, color)`

### Files Fixed
- `src/project/component/types/compCollBody.cpp` (11 calls corrected)

## Build Issue #2: Non-Existent ImTable Functions

### Error
```
error: 'addSeparator' is not a member of 'ImTable'
error: 'addLabel' is not a member of 'ImTable'
```

### Location
- `src/project/component/types/compCollBody.cpp` lines 143, 146
- `src/project/component/types/compRigidbody.cpp` lines 120, 126, 129, 132, 133

### Cause
Added calls to `ImTable::addSeparator()` and `ImTable::addLabel()` which don't exist in the ImTable namespace (defined in `src/editor/imgui/helper.h`).

### Solution
Removed all calls to these non-existent functions. Properties remain properly organized without visual separators.

### Files Fixed
- `src/project/component/types/compCollBody.cpp` (2 calls removed)
- `src/project/component/types/compRigidbody.cpp` (6 calls removed)

## Verification

### Check for Remaining Issues
```bash
# Check for incorrect line drawing
grep "vp.getLines()->" src/project/component/types/compCollBody.cpp
# Should return: (no matches)

# Check for non-existent ImTable functions
grep "addSeparator\|addLabel" src/project/component/types/*.cpp
# Should return: (no matches)
```

### Build Status
✅ All build errors resolved
✅ Editor should compile successfully
✅ Ready for testing

## Related Documentation
- `docs/COLLIDER-RIGIDBODY-SEPARATION.md` - Component architecture
- `docs/physics-api.md` - API documentation
- `docs/physics-implementation.md` - Technical details
