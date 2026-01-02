#include "physics/intersect.hpp"

#include "physics/contact.hpp"

bool Physics::Intersect(Body& a, Body& b, f32 delta_seconds, Contact& contact)
{
    // PHYSICS_ASSERT(a.shape->GetType() == ShapeType::Sphere && b.shape->GetType() == ShapeType::Sphere,
    //                "We only support intersect of two spheres!");
    contact.a = &a;
    contact.b = &b;

    if (a.shape->GetType() == ShapeType::Sphere && b.shape->GetType() == ShapeType::Sphere)
    {
        const SphereShape* sphere_a = static_cast<const SphereShape*>(a.shape);
        const SphereShape* sphere_b = static_cast<const SphereShape*>(b.shape);

        const Vector3r position_a = a.position;
        const Vector3r position_b = b.position;
        const Vector3r velocity_a = a.linear_velocity;
        const Vector3r velocity_b = b.linear_velocity;

        if (SphereSphereDynamic(sphere_a, sphere_b, position_a, position_b, velocity_a, velocity_b, delta_seconds,
                                contact.point_on_a_world_space, contact.point_on_b_world_space, contact.time_of_impact))
        {
            // Advance simulation until the time of impact on these bodies
            a.Update(contact.time_of_impact);
            b.Update(contact.time_of_impact);
            contact.point_on_a_local_space = a.WorldSpaceToBodySpace(contact.point_on_a_world_space);
            contact.point_on_b_local_space = b.WorldSpaceToBodySpace(contact.point_on_b_world_space);
            contact.normal = Opal::Normalize(b.position - a.position);

            // Undo the simulation until the time of impact
            a.Update(-contact.time_of_impact);
            b.Update(-contact.time_of_impact);

            // Calculate the separation distance
            const Vector3r ab = b.position - a.position;
            contact.separation_distance = static_cast<real>(Opal::Length(ab)) - (sphere_a->GetRadius() + sphere_b->GetRadius());
            return true;
        }
    }

    return false;
}

bool Physics::RaySphereIntersect(const Vector3r& ray_start, const Vector3r& ray_direction, const Vector3r& sphere_center,
                                 real sphere_radius, f32& t1, f32& t2)
{
    const Vector3r m = sphere_center - ray_start;
    const real a = Opal::Dot(ray_direction, ray_direction);
    const real b = Opal::Dot(m, ray_direction);
    const real c = Opal::Dot(m, m) - (sphere_radius * sphere_radius);
    const real delta = (b * b) - (a * c);
    const real inverse_a = PHYSICS_CONST(1.0) / a;
    if (delta < 0)
    {
        // No real solutions
        return false;
    }
    const real delta_root = Opal::Sqrt(delta);
    t1 = inverse_a * (b - delta_root);
    t2 = inverse_a * (b + delta_root);
    return true;
}

bool Physics::SphereSphereDynamic(const SphereShape* shape_a, const SphereShape* shape_b, const Vector3r& position_a,
                                  const Vector3r& position_b, const Vector3r& velocity_a, const Vector3r& velocity_b, f32 delta_seconds,
                                  Vector3r& point_on_a, Vector3r& point_on_b, f32& time_of_impact)
{
    const Vector3r relative_velocity = velocity_a - velocity_b;

    const Vector3r start_point_a = position_a;
    const Vector3r end_point_a = position_a + relative_velocity * delta_seconds;
    const Vector3r ray_direction = end_point_a - start_point_a;

    f32 t0 = 0.0f;
    f32 t1 = 0.0f;
    constexpr real k_kinda_small_number = PHYSICS_CONST(0.001);
    if (Opal::LengthSquared(ray_direction) < k_kinda_small_number * k_kinda_small_number)
    {
        // Ray is too short, just check if spheres intersect
        const Vector3r ab = position_b - position_a;
        const real radius = shape_a->GetRadius() + shape_b->GetRadius() + k_kinda_small_number;
        if (Opal::LengthSquared(ab) > radius * radius)
        {
            // Spheres don't intersect
            return false;
        }
    }
    else if (!RaySphereIntersect(position_a, ray_direction, position_b, shape_a->GetRadius() + shape_b->GetRadius(), t0, t1))
    {
        // Sphere paths don't intersect
        return false;
    }

    // Convert from [0, 1] to [0, delta_seconds]
    t0 *= delta_seconds;
    t1 *= delta_seconds;

    // If collisions are only in the past it means that there are no collisions this frame
    if (t1 < 0)
    {
        return false;
    }

    // Get the earliest positive time of impact
    time_of_impact = t0 < 0 ? 0 : t0;
    if (time_of_impact > delta_seconds)
    {
        // Collision is too far into the future, so we ignore it this frame
        return false;
    }

    const Vector3r new_position_a = position_a + time_of_impact * velocity_a;
    const Vector3r new_position_b = position_b + time_of_impact * velocity_b;
    const Vector3r ab = Opal::Normalize(new_position_b - new_position_a);
    point_on_a = new_position_a + ab * shape_a->GetRadius();
    point_on_b = new_position_b - ab * shape_b->GetRadius();
    return true;
}
