/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#pragma once
#include "physics/shapes.h"
#include "scene/object.h"

namespace P64::Comp
{
  struct CollBody;
}

namespace P64::Physics
{
  /**
   * Runtime API for managing multiple collision shapes on a CollBody component
   */
  class CollBodyShapeBuilder {
  public:
    /**
     * Enable new physics system for a CollBody
     * This must be called before adding shapes
     */
    static void enableNewPhysics(Comp::CollBody* body);
    
    /**
     * Add a sphere shape to the body
     */
    static void addSphere(Comp::CollBody* body, float radius, const fm_vec3_t& localOffset = {0,0,0});
    
    /**
     * Add a box shape to the body
     */
    static void addBox(Comp::CollBody* body, const fm_vec3_t& halfSize, const fm_vec3_t& localOffset = {0,0,0});
    
    /**
     * Add a cylinder shape to the body
     */
    static void addCylinder(Comp::CollBody* body, float radius, float halfHeight, const fm_vec3_t& localOffset = {0,0,0});
    
    /**
     * Add a capsule shape to the body
     */
    static void addCapsule(Comp::CollBody* body, float radius, float innerHalfHeight, const fm_vec3_t& localOffset = {0,0,0});
    
    /**
     * Clear all shapes from the body
     */
    static void clearShapes(Comp::CollBody* body);
    
    /**
     * Set physics properties
     */
    static void setMass(Comp::CollBody* body, float mass);
    static void setFriction(Comp::CollBody* body, float friction);
    static void setBounce(Comp::CollBody* body, float bounce);
    static void setKinematic(Comp::CollBody* body, bool isKinematic);
  };
}
