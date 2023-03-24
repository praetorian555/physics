#pragma once

#include "physics/containers.h"

namespace physics
{

class RigidBody;

/**
 * A potential contact between two rigid bodies.
 */
struct PotentialContact
{
    RigidBody* first_body;
    RigidBody* second_body;
};

namespace BroadPhase
{

/**
 * Node in the bounding volume hierarchy.
 * @tparam BoundingVolume Type of the bounding volume used.
 */
template <typename BoundingVolume>
class BVHNode
{
    BVHNode* left = nullptr;
    BVHNode* right = nullptr;
    BVHNode* parent = nullptr;

    BoundingVolume bounding_volume;

    /** If node doesn't have a body its not a leaf node. */
    RigidBody* body = nullptr;

    /**
     * Removes the node from the hierarchy and recalculates the bounding volume of the parent node.
     */
    ~BVHNode();

    bool IsLeaf() const { return body != nullptr; }

    /**
     * Insert a rigid body into the bounding volume hierarchy.
     * @param in_body Rigid body to insert.
     * @param in_bounding_volume Bounding volume of the rigid body.
     */
    void Insert(RigidBody* in_body, const BoundingVolume& in_bounding_volume);
};

/**
 * Collect potential contacts in children of the bounding volume hierarchy node.
 * @tparam BoundingVolume Bounding volume type.
 * @param in_root_node Root node of the tree to collect potential contacts from.
 * @param out_contacts Array to store potential contacts in. This function will append to the end of
 * the array.
 * @param in_capacity Maximum number of potential contacts to collect.
 * @return Returns the number of potential contacts collected.
 */
template <typename BoundingVolume>
int GetPotentialContacts(BVHNode<BoundingVolume>* in_root_node,
                         Array<PotentialContact>& out_contacts,
                         int in_capacity);

}

// Private function declarations ///////////////////////////////////////////////////////////////////

namespace BroadPhasePrivate
{

}

// Template implementations ////////////////////////////////////////////////////////////////////////



}  // namespace physics
