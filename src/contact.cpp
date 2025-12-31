#include "physics/contact.hpp"

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
