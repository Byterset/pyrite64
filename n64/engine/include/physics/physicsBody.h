/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#pragma once
#include <t3d/t3dmath.h>
#include <vector>
#include "shapes.h"

namespace P64
{
  class Object;
}

namespace P64::Physics
{
  /**
   * Physics body attached to a game object
   * Manages physics state and multiple collision shapes
   */
  struct PhysicsBody {
    Object* object{nullptr};
    
    // Dynamic properties
    fm_vec3_t velocity{};
    fm_vec3_t angularVelocity{};
    fm_vec3_t acceleration{};
    
    // Mass properties
    float mass{1.0f};
    float invMass{1.0f};         // 1/mass (cached)
    
    // Damping
    float linearDamping{0.99f};
    float angularDamping{0.98f};
    
    // Material properties
    float friction{0.5f};
    float bounce{0.0f};          // Coefficient of restitution
    
    // Flags
    bool hasGravity{true};
    bool isKinematic{false};     // Kinematic bodies don't respond to forces
    bool isTrigger{false};       // Trigger bodies don't resolve collisions
    bool isGrounded{false};      // Touching ground
    
    // Collision shapes (can have multiple)
    std::vector<ColliderShape> shapes;
    
    // Collision layers (bitmask)
    uint8_t maskRead{0xFF};      // What layers this body can collide with
    uint8_t maskWrite{0xFF};     // What layer this body is on
    
    /**
     * Initialize the physics body
     */
    void init(Object* obj, float bodyMass);
    
    /**
     * Add a collision shape to this body
     */
    void addShape(const ColliderShape& shape);
    
    /**
     * Clear all shapes
     */
    void clearShapes();
    
    /**
     * Get world-space AABB encompassing all shapes
     */
    void getWorldAABB(fm_vec3_t& outMin, fm_vec3_t& outMax) const;
    
    /**
     * Apply linear impulse to center of mass
     */
    void applyLinearImpulse(const fm_vec3_t& impulse);
    
    /**
     * Apply angular impulse
     */
    void applyAngularImpulse(const fm_vec3_t& impulse);
    
    /**
     * Integrate velocity (acceleration -> velocity)
     */
    void integrateVelocity(float deltaTime);
    
    /**
     * Integrate position (velocity -> position)
     */
    void integratePosition(float deltaTime);
  };
}
