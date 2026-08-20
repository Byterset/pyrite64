# Collider

```{image} /_static/img/ui_comp_coll_body.png
:align: center
```

A primitive-shape collider attached to the object.\
Unlike a {doc}`Collision-Mesh <collMesh>` this uses a simple analytic shape,\
which is cheap and suitable for moving/dynamic objects.\
Pair it with a {doc}`Rigid-Body <rigidBody>` for full physics simulation.

## Options

| Option | Description |
|--------|-------------|
| **Type** | The collider shape:<br>• **Box**<br>• **Sphere**<br>• **Cylinder**<br>• **Capsule**<br>• **Cone**<br>• **Pyramid** |
| **Shape size** | The dimensions for the chosen shape in meters, e.g. *Half Size* for a box, *Radius* for a sphere, or *Radius* plus *Half Height* for cylinders/capsules/cones. Scaled by the object's scale. |
| **Offset** | Offset of the shape's center relative to the object's origin, in meters. Scaled by the object's scale. |
| **Trigger** | When enabled, the collider reports overlaps as events but produces no physical (push-back) response. |
| **Reacts to** | The collision layers this body reads (which layers it collides with). |
| **Is Affecting** | The collision layers this body writes (which layers see it). |
| **Friction** | Surface friction, `0` to `1`. |
| **Bounce** | Restitution / bounciness, `0` to `1`. |

## Resizing at runtime

The size set here is the *unscaled* half extend, the object's scale is applied on top of it.
Change it from a script through the component, which re-applies the object scale and refreshes
the collider's AABB:

```cpp
auto collBody = obj.getComponent<Comp::CollBody>();

auto halfExtend = collBody->getHalfExtend();
halfExtend.y += 0.5f; // e.g. the half height of a cylinder/capsule/cone
collBody->setHalfExtend(halfExtend);
```

This function can be used regardless of what shape the collider has.
Shapes that don't use all three axes fold them in, e.g. a cylinder takes its radius from
`max(x, z)` and its half height from `y`. A sphere will determine its
radius by the maximum component of the `halfExtend`, etc.

To set the final (already scaled) size directly, or to change the shape type, use the setters on
{cpp:struct}`P64::Coll::Collider` itself, e.g. `setCylinderShape()`. Note that those get
overwritten by the component as soon as the object is scaled, `setHalfExtend()` is the permanent
version. **Offset** has its own setter, `setParentOffset()`, and is not cached by the component.

All of them keep the AABB and the mass properties of an attached {doc}`Rigid-Body <rigidBody>`
in sync and wake up sleeping bodies the change may touch, writing to a shape directly is not
possible for that reason.

## See also

- {doc}`Collision & Physics <../collision>`: general collision & physics docs.
- {doc}`Rigid-Body <rigidBody>`: add physics simulation to a collider.
- {cpp:struct}`P64::Comp::CollBody`: the runtime component in the C++ API.
