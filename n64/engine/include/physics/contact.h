/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#pragma once
#include <t3d/t3dmath.h>
#include <cstdint>

namespace P64
{
  class Object;
}

namespace P64::Physics
{
  constexpr int MAX_CONTACT_POINTS_PER_PAIR = 4;
  
  // Contact pair ID: unique combination of two object IDs
  using ContactPairId = uint32_t;
  
  /**
   * Create a unique contact pair ID from two object IDs
   */
  inline ContactPairId makeContactPairId(uint16_t idA, uint16_t idB) {
    if (idA < idB) {
      return (static_cast<ContactPairId>(idA) << 16) | idB;
    } else {
      return (static_cast<ContactPairId>(idB) << 16) | idA;
    }
  }
  
  /**
   * Single contact point data within a contact constraint
   */
  struct ContactPoint {
    fm_vec3_t point{};           // Contact point in world space
    fm_vec3_t contactA{};        // Contact on A's surface (world)
    fm_vec3_t contactB{};        // Contact on B's surface (world)
    fm_vec3_t localPointA{};     // Contact in A's local space
    fm_vec3_t localPointB{};     // Contact in B's local space
    fm_vec3_t aToContact{};      // Vector from A's center to contact
    fm_vec3_t bToContact{};      // Vector from B's center to contact
    float penetration{};         // Penetration depth
    
    // Cached solver data (for warm starting)
    float accumulatedNormalImpulse{};
    float accumulatedTangentImpulseU{};
    float accumulatedTangentImpulseV{};
    float normalMass{};          // Cached effective mass for normal
    float tangentMassU{};        // Cached effective mass for tangent U
    float tangentMassV{};        // Cached effective mass for tangent V
    float velocityBias{};        // Restitution velocity bias
    
    bool active{false};          // Was this point active this frame?
  };
  
  /**
   * Contact constraint containing multiple contact points for a pair of objects
   */
  struct ContactConstraint {
    Object* objectA{nullptr};
    Object* objectB{nullptr};
    
    // Shared contact data
    fm_vec3_t normal{};          // Normal from B toward A
    fm_vec3_t tangentU{};        // First tangent direction for friction
    fm_vec3_t tangentV{};        // Second tangent direction for friction
    
    // Material properties
    float combinedFriction{0.3f};
    float combinedBounce{0.0f};
    
    ContactPairId pairId{};
    int nextSamePidIndex{-1};    // Linked list for same PID
    int pointCount{0};           // Number of active points
    
    bool isActive{false};        // Found this frame?
    bool isTrigger{false};       // Is trigger collision?
    
    ContactPoint points[MAX_CONTACT_POINTS_PER_PAIR];
  };
}
