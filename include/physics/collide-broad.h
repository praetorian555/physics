#pragma once

#include "physics/containers.h"
#include "physics/shapes-helpers.h"
#include "physics/shapes-overlaps.h"

namespace Physics
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
struct BVHNode
{
    BVHNode* left = nullptr;
    BVHNode* right = nullptr;
    BVHNode* parent = nullptr;

    BoundingVolume bounding_volume;

    /** If node doesn't have a body its not a leaf node. */
    RigidBody* body = nullptr;

    BVHNode(BVHNode* in_parent, const BoundingVolume& in_bounding_volume, RigidBody* in_body);

    /**
     * Removes the node from the hierarchy and recalculates the bounding volume of the parent node.
     */
    ~BVHNode();

    /**
     * Check if the node is a leaf node, which means it has a rigid body.
     * @return Returns true if the node is a leaf node, false otherwise.
     */
    [[nodiscard]] bool IsLeaf() const { return body != nullptr; }

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
                         Array<PotentialContact>& out_contacts);

// Private function declarations ///////////////////////////////////////////////////////////////////

namespace Private
{

template <typename BoundingVolume>
bool ShouldDescendLeft(BVHNode<BoundingVolume>* left, BVHNode<BoundingVolume>* right);

template <typename BoundingVolume>
bool ShouldInsertLeft(BVHNode<BoundingVolume>* left,
                      BVHNode<BoundingVolume>* right,
                      const BoundingVolume& new_volume);

/**
 * Recalculate the bounding volume of the node and all its parents.
 * @tparam BoundingVolume Type of the bounding volume.
 * @param node Node to recalculate the bounding volume of.
 */
template <typename BoundingVolume>
void RecalculateBoundingVolume(BVHNode<BoundingVolume>* node);

}  // namespace Private

}  // namespace BroadPhase

}  // namespace Physics

// Template implementations ////////////////////////////////////////////////////////////////////////

template <typename BoundingVolume>
Physics::BroadPhase::BVHNode<BoundingVolume>::BVHNode(BVHNode* in_parent,
                                                      const BoundingVolume& in_bounding_volume,
                                                      RigidBody* in_body)
    : parent(in_parent), bounding_volume(in_bounding_volume), body(in_body)
{
}

template <typename BoundingVolume>
Physics::BroadPhase::BVHNode<BoundingVolume>::~BVHNode()
{
    if (parent != nullptr)
    {
        BVHNode* sibling = parent->left == this ? parent->right : parent->left;
        parent->left = sibling->left;
        parent->right = sibling->right;
        parent->body = sibling->body;
        parent->bounding_volume = sibling->bounding_volume;
        sibling->parent = nullptr;
        sibling->left = nullptr;
        sibling->right = nullptr;
        delete sibling;
    }

    if (left != nullptr)
    {
        left->parent = nullptr;
        delete left;
    }
    if (right != nullptr)
    {
        right->parent = nullptr;
        delete right;
    }
}

template <typename BoundingVolume>
void Physics::BroadPhase::BVHNode<BoundingVolume>::Insert(RigidBody* in_body,
                                                          const BoundingVolume& in_bounding_volume)
{
    BVHNode* node = this;
    while (node != nullptr)
    {
        if (node->IsLeaf())
        {
            left = new BVHNode<BoundingVolume>(this, bounding_volume, body);
            right = new BVHNode<BoundingVolume>(this, in_bounding_volume, in_body);
            body = nullptr;
            bounding_volume = Enclose(bounding_volume, in_bounding_volume);
            return;
        }
        if (Private::ShouldInsertLeft(node->left, node->right, in_bounding_volume))
        {
            node = node->left;
        }
        else
        {
            node = node->right;
        }
    }
}

template <typename BoundingVolume>
int Physics::BroadPhase::GetPotentialContacts(BVHNode<BoundingVolume>* in_root_node,
                                              Array<PotentialContact>& out_contacts)
{
    if (in_root_node == nullptr || in_root_node->IsLeaf())
    {
        return 0;
    }
    if (in_root_node->left == nullptr || in_root_node->right == nullptr)
    {
        return 0;
    }

    struct Pair
    {
        BVHNode<BoundingVolume>* left;
        BVHNode<BoundingVolume>* right;
    };
    Array<Pair> stack;
    stack.reserve(32);
    stack.push_back({in_root_node->left, in_root_node->right});
    while (!stack.empty())
    {
        Pair pair = stack.back();
        stack.pop_back();

        if (!Overlaps(pair.left->bounding_volume, pair.right->bounding_volume))
        {
            continue;
        }
        if (pair.left->IsLeaf() && pair.right->IsLeaf())
        {
            out_contacts.push_back({pair.left->body, pair.right->body});
            continue;
        }
        if (Private::ShouldDescendLeft(pair.left, pair.right))
        {
            stack.push_back({pair.left->right, pair.right});
            stack.push_back({pair.left->left, pair.right});
        }
        else
        {
            stack.push_back({pair.left, pair.right->right});
            stack.push_back({pair.right, pair.right->left});
        }
    }
}

template <typename BoundingVolume>
bool Physics::BroadPhase::Private::ShouldDescendLeft(BVHNode<BoundingVolume>* left,
                                                     BVHNode<BoundingVolume>* right)
{
    return right->IsLeaf() ||
           (left->bounding_volume->GetVolume() >= right->bounding_volume->GetVolume());
}

template <typename BoundingVolume>
bool Physics::BroadPhase::Private::ShouldInsertLeft(BVHNode<BoundingVolume>* left,
                                                    BVHNode<BoundingVolume>* right,
                                                    const BoundingVolume& new_volume)
{
    const BoundingVolume left_volume = Enclose(left->bounding_volume, new_volume);
    const BoundingVolume right_volume = Enclose(right->bounding_volume, new_volume);
    return left_volume.GetVolume() < right_volume.GetVolume();
}

template <typename BoundingVolume>
void Physics::BroadPhase::Private::RecalculateBoundingVolume(BVHNode<BoundingVolume>* node)
{
    if (node == nullptr || node->IsLeaf())
    {
        return;
    }

    Array<BVHNode<BoundingVolume>*> stack;
    stack.reserve(16);
    stack.push_back(node);
    while (!stack.empty())
    {
        BVHNode<BoundingVolume>* current = stack.back();
        stack.pop_back();

        current->bounding_volume =
            Enclose(current->left->bounding_volume, current->right->bounding_volume);
        if (current->parent != nullptr)
        {
            stack.push_back(current->parent);
        }
    }
}
