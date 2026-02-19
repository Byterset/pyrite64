/**
 * Example: Using Multi-Shape Physics System
 * 
 * This example demonstrates how to enable the new physics system
 * and create a character with multiple collision shapes.
 */

#include "scene/components/collBody.h"
#include "physics/collBodyBuilder.h"

// Example: Creating a character with capsule body + sphere head
void setupPlayerPhysics(P64::Object* playerObject) 
{
    // Get existing CollBody component (assumes it was added in editor or at init)
    auto* collBody = playerObject->getComponent<P64::Comp::CollBody>();
    if (!collBody) return;
    
    // Enable new physics system (disables legacy BCS)
    P64::Physics::CollBodyShapeBuilder::enableNewPhysics(collBody);
    
    // Clear default shape
    P64::Physics::CollBodyShapeBuilder::clearShapes(collBody);
    
    // Add capsule for main body (1 unit radius, 2 units tall)
    P64::Physics::CollBodyShapeBuilder::addCapsule(
        collBody,
        1.0f,  // radius
        1.0f,  // inner half height (cylinder part)
        {0, 1.5f, 0}  // offset upward
    );
    
    // Add sphere for head (0.8 unit radius)
    P64::Physics::CollBodyShapeBuilder::addSphere(
        collBody,
        0.8f,  // radius
        {0, 3.5f, 0}  // offset above body
    );
    
    // Set physics properties
    P64::Physics::CollBodyShapeBuilder::setMass(collBody, 70.0f);
    P64::Physics::CollBodyShapeBuilder::setFriction(collBody, 0.5f);
    P64::Physics::CollBodyShapeBuilder::setBounce(collBody, 0.0f);
}

// Example: Creating a vehicle with box body + cylinder wheels
void setupVehiclePhysics(P64::Object* vehicleObject)
{
    auto* collBody = vehicleObject->getComponent<P64::Comp::CollBody>();
    if (!collBody) return;
    
    P64::Physics::CollBodyShapeBuilder::enableNewPhysics(collBody);
    P64::Physics::CollBodyShapeBuilder::clearShapes(collBody);
    
    // Main body (box)
    P64::Physics::CollBodyShapeBuilder::addBox(
        collBody,
        {2.0f, 1.0f, 4.0f},  // half extents (width, height, length)
        {0, 1.0f, 0}  // centered, elevated
    );
    
    // Front wheels (cylinders, X-axis aligned would require rotation support)
    // For now, using small boxes as wheel approximations
    P64::Physics::CollBodyShapeBuilder::addBox(
        collBody,
        {0.3f, 0.5f, 0.5f},  // wheel size
        {-1.8f, 0.5f, 2.0f}  // front-left
    );
    
    P64::Physics::CollBodyShapeBuilder::addBox(
        collBody,
        {0.3f, 0.5f, 0.5f},
        {1.8f, 0.5f, 2.0f}  // front-right
    );
    
    // Rear wheels
    P64::Physics::CollBodyShapeBuilder::addBox(
        collBody,
        {0.3f, 0.5f, 0.5f},
        {-1.8f, 0.5f, -2.0f}  // rear-left
    );
    
    P64::Physics::CollBodyShapeBuilder::addBox(
        collBody,
        {0.3f, 0.5f, 0.5f},
        {1.8f, 0.5f, -2.0f}  // rear-right
    );
    
    P64::Physics::CollBodyShapeBuilder::setMass(collBody, 1000.0f);
    P64::Physics::CollBodyShapeBuilder::setFriction(collBody, 0.7f);
}

// Example: Creating a crate with just a box (simpler case)
void setupCratePhysics(P64::Object* crateObject)
{
    auto* collBody = crateObject->getComponent<P64::Comp::CollBody>();
    if (!collBody) return;
    
    P64::Physics::CollBodyShapeBuilder::enableNewPhysics(collBody);
    P64::Physics::CollBodyShapeBuilder::clearShapes(collBody);
    
    // Single box shape
    P64::Physics::CollBodyShapeBuilder::addBox(
        collBody,
        {1.0f, 1.0f, 1.0f},  // 2x2x2 cube
        {0, 0, 0}  // centered
    );
    
    P64::Physics::CollBodyShapeBuilder::setMass(collBody, 50.0f);
    P64::Physics::CollBodyShapeBuilder::setFriction(collBody, 0.6f);
    P64::Physics::CollBodyShapeBuilder::setBounce(collBody, 0.1f);
}

// Example: Making an object kinematic (doesn't respond to physics)
void setupStaticPlatform(P64::Object* platformObject)
{
    auto* collBody = platformObject->getComponent<P64::Comp::CollBody>();
    if (!collBody) return;
    
    P64::Physics::CollBodyShapeBuilder::enableNewPhysics(collBody);
    P64::Physics::CollBodyShapeBuilder::clearShapes(collBody);
    
    // Large platform
    P64::Physics::CollBodyShapeBuilder::addBox(
        collBody,
        {10.0f, 0.5f, 10.0f},
        {0, 0, 0}
    );
    
    // Make it kinematic so it doesn't fall or move
    P64::Physics::CollBodyShapeBuilder::setKinematic(collBody, true);
}

// Example: Trigger volume (doesn't resolve collisions, just detects)
void setupTriggerZone(P64::Object* triggerObject)
{
    auto* collBody = triggerObject->getComponent<P64::Comp::CollBody>();
    if (!collBody) return;
    
    P64::Physics::CollBodyShapeBuilder::enableNewPhysics(collBody);
    P64::Physics::CollBodyShapeBuilder::clearShapes(collBody);
    
    // Large trigger box
    P64::Physics::CollBodyShapeBuilder::addBox(
        collBody,
        {5.0f, 3.0f, 5.0f},
        {0, 0, 0}
    );
    
    // Triggers are detected but don't resolve (no physics response)
    // This is controlled via the isTrigger flag set during init
}

// Note: Legacy collision system usage (unchanged)
void legacyExample(P64::Object* object)
{
    // Objects without enableNewPhysics() continue using legacy BCS system
    auto* collBody = object->getComponent<P64::Comp::CollBody>();
    
    // This still works exactly as before - no changes needed
    // Legacy system is the default for backward compatibility
}
