### Collision handling

Colliders for a body are defined in its local space.
In this space, the center of mass of the body is always at the origin, even if physically incorrect.
Otherwise, when adding or removing colliders, the body's position would change or its colliders would be shifted.

When bodies change position at the end of a time step, its transform is updated accordingly.
In turn, its colliders are updated as well as the bounding box in the BVH tree.

When treating collisions between colliders, both colliders are expected to be in world space.

### Colliders

The following colliders for objects are implemented:

- BoundingBox
- Circle
- Capsule
- Polygon

The bounding box is always axis-aligned.
It can be translated but cannot be rotated.
It is used extensively in the BVH tree to prune impossible collisions.
It cannot be used as an object's collider, use `Polygon::box` instead.

The capsule can be translated and rotated.
It is well suited as an approximation of complex shapes.

The polygon can be translated and rotated.
It must be convex and has a limited number of vertices to avoid allocating vertices separately.

