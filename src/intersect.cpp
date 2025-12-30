#include "physics/intersect.hpp"

bool Physics::Intersect(Body& a, Body& b, f32 delta_seconds, Contact& contact)
{
    PHYSICS_ASSERT(a.shape->GetType() == ShapeType::Sphere && b.shape->GetType() == ShapeType::Sphere,
                   "We only support intersect of two spheres!");
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

void Physics::ResolveContact(Contact& contact)
{
    Body& a = contact.a;
    Body& b = contact.b;

#ifdef OPAL_DEBUG
    contact.a_prev_position = a.position;
    contact.b_prev_position = b.position;
    contact.a_prev_linear_velocity = a.linear_velocity;
    contact.b_prev_linear_velocity = b.linear_velocity;
#endif

    const Vector3r n = contact.normal;
    const real elasticity = a.elasticity * b.elasticity;
    const Vector3r ra = contact.point_on_a_world_space - a.GetCenterOfMassWorldSpace();
    const Vector3r rb = contact.point_on_b_world_space - b.GetCenterOfMassWorldSpace();
    const Matrix3x3r inverse_inertia_a_world = a.GetInverseInertiaTensorWorldSpace();
    const Matrix3x3r inverse_inertia_b_world = b.GetInverseInertiaTensorWorldSpace();

    // Apply impulse on both bodies due to collision
    const Vector3r impulse_angular_a = Opal::Cross(inverse_inertia_a_world * Opal::Cross(ra, n), ra);
    const Vector3r impulse_angular_b = Opal::Cross(inverse_inertia_b_world * Opal::Cross(rb, n), rb);
    const real angular_factor = Opal::Dot(impulse_angular_a + impulse_angular_b, n);
    const Vector3r va = a.linear_velocity + Opal::Cross(a.angular_velocity, ra);
    const Vector3r vb = b.linear_velocity + Opal::Cross(b.angular_velocity, rb);
    const Vector3r vab = va - vb;
    const real impulse_scalar = (PHYSICS_CONST(1.0) + elasticity) * Opal::Dot(vab, n) / (a.inverse_mass + b.inverse_mass + angular_factor);
    const Vector3r impulse = impulse_scalar * n;
    a.ApplyImpulse(-impulse, contact.point_on_a_world_space);
    b.ApplyImpulse(impulse, contact.point_on_b_world_space);

    // Apply friction impulse
    const real friction = a.friction * b.friction;
    const Vector3r vab_normal = Opal::Dot(vab, n) * n;
    const Vector3r vab_tangent = vab - vab_normal;
    const Vector3r vab_tangent_relative = Opal::Normalize(vab_tangent);
    const Vector3r inertia_a = Opal::Cross(inverse_inertia_a_world * Opal::Cross(ra, vab_tangent_relative), ra);
    const Vector3r inertia_b = Opal::Cross(inverse_inertia_b_world * Opal::Cross(rb, vab_tangent_relative), rb);
    const real inverse_inertia = Opal::Dot(inertia_a + inertia_b, vab_tangent_relative);
    const real reduced_mass = PHYSICS_CONST(1.0) / (a.inverse_mass + b.inverse_mass + inverse_inertia);
    const Vector3r friction_impulse = vab_tangent * reduced_mass * friction;
    a.ApplyImpulse(-friction_impulse, contact.point_on_a_world_space);
    b.ApplyImpulse(friction_impulse, contact.point_on_b_world_space);

    // Move bodies so they don't penetrate one another.
    // When are using continuous collision detection, only time the interpenetration can happen is if this is the case at the start
    // of the frame.
    if (0.0f == contact.time_of_impact)
    {
        const Vector3r dist = contact.point_on_b_world_space - contact.point_on_a_world_space;
        a.position += dist * a.inverse_mass / (a.inverse_mass + b.inverse_mass);
        b.position -= dist * b.inverse_mass / (b.inverse_mass + a.inverse_mass);
    }
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
