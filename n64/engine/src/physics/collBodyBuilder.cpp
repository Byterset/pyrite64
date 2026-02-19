/**
 * @copyright 2026 - Max Bebök
 * @license MIT
 */
#include "physics/collBodyBuilder.h"
#include "scene/components/collBody.h"

namespace P64::Physics
{
  void CollBodyShapeBuilder::enableNewPhysics(Comp::CollBody* body) {
    if (!body || body->useNewPhysics) return;
    
    body->useNewPhysics = true;
    
    if (!body->physicsBody) {
      body->physicsBody = new PhysicsBody();
      body->physicsBody->init(body->bcs.obj, 1.0f);
      body->physicsBody->maskRead = body->bcs.maskRead;
      body->physicsBody->maskWrite = body->bcs.maskWrite;
      body->physicsBody->isTrigger = body->bcs.isTrigger();
    }
  }
  
  void CollBodyShapeBuilder::addSphere(Comp::CollBody* body, float radius, const fm_vec3_t& localOffset) {
    if (!body || !body->useNewPhysics || !body->physicsBody) return;
    
    ColliderShape shape;
    shape.type = ShapeType::SPHERE;
    shape.data.sphere.radius = radius;
    shape.localOffset = localOffset;
    
    body->physicsBody->addShape(shape);
  }
  
  void CollBodyShapeBuilder::addBox(Comp::CollBody* body, const fm_vec3_t& halfSize, const fm_vec3_t& localOffset) {
    if (!body || !body->useNewPhysics || !body->physicsBody) return;
    
    ColliderShape shape;
    shape.type = ShapeType::BOX;
    shape.data.box.halfSize = halfSize;
    shape.localOffset = localOffset;
    
    body->physicsBody->addShape(shape);
  }
  
  void CollBodyShapeBuilder::addCylinder(Comp::CollBody* body, float radius, float halfHeight, const fm_vec3_t& localOffset) {
    if (!body || !body->useNewPhysics || !body->physicsBody) return;
    
    ColliderShape shape;
    shape.type = ShapeType::CYLINDER;
    shape.data.cylinder.radius = radius;
    shape.data.cylinder.halfHeight = halfHeight;
    shape.localOffset = localOffset;
    
    body->physicsBody->addShape(shape);
  }
  
  void CollBodyShapeBuilder::addCapsule(Comp::CollBody* body, float radius, float innerHalfHeight, const fm_vec3_t& localOffset) {
    if (!body || !body->useNewPhysics || !body->physicsBody) return;
    
    ColliderShape shape;
    shape.type = ShapeType::CAPSULE;
    shape.data.capsule.radius = radius;
    shape.data.capsule.innerHalfHeight = innerHalfHeight;
    shape.localOffset = localOffset;
    
    body->physicsBody->addShape(shape);
  }
  
  void CollBodyShapeBuilder::clearShapes(Comp::CollBody* body) {
    if (!body || !body->useNewPhysics || !body->physicsBody) return;
    body->physicsBody->clearShapes();
  }
  
  void CollBodyShapeBuilder::setMass(Comp::CollBody* body, float mass) {
    if (!body || !body->useNewPhysics || !body->physicsBody) return;
    body->physicsBody->mass = mass;
    body->physicsBody->invMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
  }
  
  void CollBodyShapeBuilder::setFriction(Comp::CollBody* body, float friction) {
    if (!body || !body->useNewPhysics || !body->physicsBody) return;
    body->physicsBody->friction = friction;
  }
  
  void CollBodyShapeBuilder::setBounce(Comp::CollBody* body, float bounce) {
    if (!body || !body->useNewPhysics || !body->physicsBody) return;
    body->physicsBody->bounce = bounce;
  }
  
  void CollBodyShapeBuilder::setKinematic(Comp::CollBody* body, bool isKinematic) {
    if (!body || !body->useNewPhysics || !body->physicsBody) return;
    body->physicsBody->isKinematic = isKinematic;
  }
}
