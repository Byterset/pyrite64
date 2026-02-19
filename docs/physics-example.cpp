/**
 * Example: Using Multi-Shape Rigidbody System
 * 
 * This example demonstrates how to use rigidbodies with multiple collision shapes.
 * Note: Only ONE rigidbody per object is allowed!
 */

#include "scene/components/collBody.h"

// Example: Creating a character with capsule body + sphere head
void setupPlayerPhysics(P64::Object* playerObject) 
{
    // Get the Rigidbody component (must be added in editor or during creation)
    auto* rigidbody = playerObject->getComponent<P64::Comp::CollBody>();
    if (!rigidbody) return;
    
    // Clear default shape
    rigidbody->clearShapes();
    
    // Add capsule for main body (1 unit radius, 2 units tall)
    rigidbody->addCapsule(
        1.0f,  // radius
        1.0f,  // inner half height (cylinder part)
        {0, 1.5f, 0}  // offset upward
    );
    
    // Add sphere for head (0.8 unit radius)
    rigidbody->addSphere(
        0.8f,  // radius
        {0, 3.5f, 0}  // offset above body
    );
    
    // Set physics properties
    rigidbody->setMass(70.0f);
    rigidbody->setFriction(0.5f);
    rigidbody->setBounce(0.0f);
}

// Example: Creating a vehicle with box body + wheel approximations
void setupVehiclePhysics(P64::Object* vehicleObject)
{
    auto* rigidbody = vehicleObject->getComponent<P64::Comp::CollBody>();
    if (!rigidbody) return;
    
    rigidbody->clearShapes();
    
    // Main body (box)
    rigidbody->addBox(
        {2.0f, 1.0f, 4.0f},  // half extents (width, height, length)
        {0, 1.0f, 0}  // centered, elevated
    );
    
    // Front wheels (using small boxes as wheel approximations)
    rigidbody->addBox(
        {0.3f, 0.5f, 0.5f},  // wheel size
        {-1.8f, 0.5f, 2.0f}  // front-left
    );
    
    rigidbody->addBox(
        {0.3f, 0.5f, 0.5f},
        {1.8f, 0.5f, 2.0f}  // front-right
    );
    
    // Rear wheels
    rigidbody->addBox(
        {0.3f, 0.5f, 0.5f},
        {-1.8f, 0.5f, -2.0f}  // rear-left
    );
    
    rigidbody->addBox(
        {0.3f, 0.5f, 0.5f},
        {1.8f, 0.5f, -2.0f}  // rear-right
    );
    
    rigidbody->setMass(1000.0f);
    rigidbody->setFriction(0.7f);
}

// Example: Creating a crate with just a box (simpler case)
void setupCratePhysics(P64::Object* crateObject)
{
    auto* rigidbody = crateObject->getComponent<P64::Comp::CollBody>();
    if (!rigidbody) return;
    
    rigidbody->clearShapes();
    
    // Single box shape
    rigidbody->addBox(
        {1.0f, 1.0f, 1.0f},  // 2x2x2 cube
        {0, 0, 0}  // centered
    );
    
    rigidbody->setMass(50.0f);
    rigidbody->setFriction(0.6f);
    rigidbody->setBounce(0.1f);
}

// Example: Making an object kinematic (doesn't respond to physics)
void setupStaticPlatform(P64::Object* platformObject)
{
    auto* rigidbody = platformObject->getComponent<P64::Comp::CollBody>();
    if (!rigidbody) return;
    
    rigidbody->clearShapes();
    
    // Large platform
    rigidbody->addBox(
        {10.0f, 0.5f, 10.0f},
        {0, 0, 0}
    );
    
    // Make it kinematic so it doesn't fall or move
    rigidbody->setKinematic(true);
}

// Example: Trigger volume (doesn't resolve collisions, just detects)
void setupTriggerZone(P64::Object* triggerObject)
{
    auto* rigidbody = triggerObject->getComponent<P64::Comp::CollBody>();
    if (!rigidbody) return;
    
    rigidbody->clearShapes();
    
    // Large trigger box
    rigidbody->addBox(
        {5.0f, 3.0f, 5.0f},
        {0, 0, 0}
    );
    
    // Triggers are detected but don't resolve (no physics response)
    // This is controlled via the isTrigger flag set during init
}

// IMPORTANT: Only one rigidbody per object!
void incorrectUsage(P64::Object* object)
{
    // WRONG - DO NOT DO THIS!
    // Trying to add a second rigidbody will cause an error
    // auto* rb1 = object->addComponent<Comp::CollBody>();  // First one - OK
    // auto* rb2 = object->addComponent<Comp::CollBody>();  // ERROR: Duplicate!
    
    // CORRECT - Add multiple shapes to ONE rigidbody
    auto* rb = object->getComponent<P64::Comp::CollBody>();
    rb->addBox({1, 1, 1}, {0, 0, 0});
    rb->addSphere(0.5f, {0, 2, 0});
    // This is how you create compound colliders!
}

