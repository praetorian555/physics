#include "physics/broad-phase.hpp"

#include <algorithm>

bool Physics::CollisionPair::operator==(const CollisionPair& other) const
{
    return (a == other.a && b == other.b) || (a == other.b && b == other.a);
}

namespace
{

struct SweepAndPruneBody
{
    Physics::i32 id;
    Physics::f32 value;
    bool is_min;
};

void SortBodiesByBounds(Opal::ArrayView<Physics::Body> bodies, Opal::ArrayView<SweepAndPruneBody> array_to_sort, Physics::f32 delta_seconds)
{
    const Physics::Vector3r axis = Opal::Normalize(Physics::Vector3r(1, 1, 1));
    for (Physics::i32 body_idx = 0; body_idx < bodies.GetSize(); ++body_idx)
    {
        const Physics::Body& body = bodies[body_idx];
        Physics::Bounds3r bounds = body.shape->GetBounds(body.position, body.orientation);
        bounds = Opal::Union(bounds, bounds.min + body.linear_velocity * delta_seconds);
        bounds = Opal::Union(bounds, bounds.max + body.linear_velocity * delta_seconds);

        constexpr Physics::real k_kinda_small_number = PHYSICS_CONST(0.01);
        bounds = Opal::Union(bounds, bounds.min + k_kinda_small_number * Physics::Vector3r(-1, -1, -1));
        bounds = Opal::Union(bounds, bounds.max + k_kinda_small_number * Physics::Vector3r(1, 1, 1));

        const Physics::i32 sorted_array_idx = 2 * body_idx;
        array_to_sort[sorted_array_idx].id = body_idx;
        array_to_sort[sorted_array_idx].value = Opal::Dot(Physics::PointToVector(bounds.min), axis);
        array_to_sort[sorted_array_idx].is_min = true;
        array_to_sort[sorted_array_idx + 1].id = body_idx;
        array_to_sort[sorted_array_idx + 1].value = Opal::Dot(Physics::PointToVector(bounds.max), axis);
        array_to_sort[sorted_array_idx + 1].is_min = false;
    }

    std::ranges::sort(array_to_sort, [](const SweepAndPruneBody& a, const SweepAndPruneBody& b)
    {
        return a.value < b.value;
    });
}

Opal::DynamicArray<Physics::CollisionPair> BuildPairs(Opal::ArrayView<SweepAndPruneBody> sorted_array)
{
    Opal::DynamicArray<Physics::CollisionPair> collision_pairs;
    for (Physics::i32 first_body_idx = 0; first_body_idx < sorted_array.GetSize(); ++first_body_idx)
    {
        const SweepAndPruneBody& a = sorted_array[first_body_idx];
        if (!a.is_min)
        {
            continue;
        }

        Physics::CollisionPair pair;
        pair.a = a.id;
        for (Physics::i32 second_body_idx = first_body_idx + 1; second_body_idx < sorted_array.GetSize(); ++second_body_idx)
        {
            const SweepAndPruneBody& b = sorted_array[second_body_idx];
            if (a.id == b.id)
            {
                // We reached the end of the body's bounds so we are done creating collisions for it
                break;
            }
            if (!b.is_min)
            {
                continue;
            }
            pair.b = b.id;
            collision_pairs.PushBack(pair);
        }
    }
    return collision_pairs;
}

Opal::DynamicArray<Physics::CollisionPair> SweepAndPrune1D(Opal::ArrayView<Physics::Body> bodies, Physics::f32 delta_seconds)
{
    Opal::AllocatorBase* scratch_allocator = Opal::GetScratchAllocator();
    Opal::DynamicArray<SweepAndPruneBody> sweep_and_prune_bodies(2 * bodies.GetSize(), scratch_allocator);
    SortBodiesByBounds(bodies, sweep_and_prune_bodies, delta_seconds);
    return BuildPairs(sweep_and_prune_bodies);
}

}

Opal::DynamicArray<Physics::CollisionPair> Physics::BroadPhase(Opal::ArrayView<Body> bodies, f32 delta_seconds)
{
    return SweepAndPrune1D(bodies, delta_seconds);
}