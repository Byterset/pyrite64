/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#pragma once
#include <vector>
#include <unordered_map>
#include "physicsBody.h"
#include "contact.h"

namespace P64::Physics
{
  constexpr int MAX_CACHED_CONTACTS = 256;
  constexpr int VELOCITY_SOLVER_ITERATIONS = 7;
  constexpr int POSITION_SOLVER_ITERATIONS = 4;
  constexpr float GRAVITY = -9.8f * 1.5f;  // N64 units scale
  
  /**
   * Main physics scene managing all physics bodies and constraint solving
   */
  class PhysicsScene {
  private:
    std::vector<PhysicsBody*> bodies;
    
    // Contact constraint cache
    ContactConstraint cachedConstraints[MAX_CACHED_CONTACTS];
    int cachedConstraintCount{0};
    
    // Hash map for contact caching (pair ID -> constraint index)
    std::unordered_map<ContactPairId, int> contactMap;
    
    // Internal phase functions
    void updateWorldInertia();
    void applyGravityAndIntegrateVelocity(float deltaTime);
    void detectAllContacts();
    void preSolveContacts();
    void warmStart();
    void solveVelocityConstraints();
    void integratePositions(float deltaTime);
    void solvePositionConstraints();
    void updateSleepStates();
    
    // Contact detection helpers
    void detectBodyVsBody(PhysicsBody* a, PhysicsBody* b);
    ContactConstraint* findOrCreateContactConstraint(Object* objA, Object* objB);
    
    // Collision detection (GJK/EPA)
    bool gjkCheckOverlap(const ColliderShape& shapeA, const fm_vec3_t& posA, const fm_quat_t& rotA,
                         const ColliderShape& shapeB, const fm_vec3_t& posB, const fm_quat_t& rotB,
                         fm_vec3_t& outNormal, float& outPenetration,
                         fm_vec3_t& outContactA, fm_vec3_t& outContactB);
    
  public:
    PhysicsScene();
    ~PhysicsScene();
    
    /**
     * Register a physics body with the scene
     */
    void registerBody(PhysicsBody* body);
    
    /**
     * Unregister a physics body from the scene
     */
    void unregisterBody(PhysicsBody* body);
    
    /**
     * Main physics step - runs full constraint solver pipeline
     */
    void step(float deltaTime);
    
    /**
     * Get all registered bodies
     */
    const std::vector<PhysicsBody*>& getBodies() const { return bodies; }
  };
}
