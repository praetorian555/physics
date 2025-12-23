#include "physics/intersect.hpp"

bool Physics::Intersect(Body& a, Body& b, Contact& contact)
{
    PHYSICS_ASSERT(a.shape->GetType() == ShapeType::Sphere && b.shape->GetType() == ShapeType::Sphere, "We only support intersect of two spheres!");
    contact.a = &a;
    contact.b = &b;
    const Vector3r dist = b.position - a.position;
    contact.normal = dist;
    contact.normal = Opal::Normalize(contact.normal);
    const SphereShape* sphere_a = static_cast<const SphereShape*>(a.shape);
    const SphereShape* sphere_b = static_cast<const SphereShape*>(b.shape);
    contact.point_on_a_world_space = a.position + sphere_a->GetRadius() * contact.normal;
    contact.point_on_b_world_space = b.position - sphere_b->GetRadius() * contact.normal;
    const real radius_sum = sphere_a->GetRadius() + sphere_b->GetRadius();
    const real distance_sq = LengthSquared(dist);
    return distance_sq <= radius_sum * radius_sum;
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
    const Vector3r vab = a.linear_velocity - b.linear_velocity;
    const real impulse_scalar = -PHYSICS_CONST(2.0) * Dot(vab, n) / (a.inverse_mass + b.inverse_mass);
    const Vector3r impulse = impulse_scalar * n;
    a.ApplyImpulseLinear(impulse);
    b.ApplyImpulseLinear(-impulse);

    const Vector3r dist = contact.point_on_b_world_space - contact.point_on_a_world_space;
    a.position += dist * a.inverse_mass / (a.inverse_mass + b.inverse_mass);
    b.position -= dist * b.inverse_mass / (b.inverse_mass + a.inverse_mass);
}
