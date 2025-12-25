#include "shared/sample-app.hpp"

class SpheresSampleApp : public SampleApp
{
public:
    void SetupSimulation() override
    {
        SetPhysicsUpdateInterval(0.016f);

        Opal::AllocatorBase* allocator = Opal::GetDefaultAllocator();

        Physics::Body body;
        body.position = Physics::Vector3r{0, 10, -10};
        body.orientation = Physics::Quatr::Identity();
        body.linear_velocity = Physics::Vector3r{1, 0, 0};
        body.inverse_mass = 1.0f;
        body.elasticity = 0.0f;
        body.friction = 0.5f;
        Opal::ScopePtr<Physics::SphereShape> sphere(allocator, PHYSICS_CONST(1.0));
        body.shape = sphere.Get();
        AddBody(body);
        m_shapes.PushBack(std::move(sphere));

        Physics::Body ground;
        ground.position = Physics::Vector3r{0, -998, -10};
        ground.orientation = Physics::Quatr::FromAxisAngleDegrees(Physics::Vector3r{1, 0, 0}, 45.0f);
        ground.inverse_mass = 0.0f;
        ground.elasticity = 1.0f;
        ground.friction = 0.5f;
        Opal::ScopePtr<Physics::SphereShape> ground_sphere(allocator, PHYSICS_CONST(1000.0));
        ground.shape = ground_sphere.Get();
        AddBody(ground);
        m_shapes.PushBack(std::move(ground_sphere));
    }

    void ResetSimulation() override
    {
        SampleApp::ResetSimulation();
        m_shapes.Clear();

        Opal::AllocatorBase* allocator = Opal::GetDefaultAllocator();

        Physics::Body body;
        body.position = Physics::Vector3r{0, 10, -10};
        body.orientation = Physics::Quatr::Identity();
        body.inverse_mass = 1.0f;
        Opal::ScopePtr<Physics::SphereShape> sphere(allocator, PHYSICS_CONST(1.0));
        body.shape = sphere.Get();
        AddBody(body);
        m_shapes.PushBack(std::move(sphere));

        Physics::Body ground;
        ground.position = Physics::Vector3r{0, -998, -10};
        ground.orientation = Physics::Quatr::FromAxisAngleDegrees(Physics::Vector3r{1, 0, 0}, 45.0f);
        ground.inverse_mass = 0.0f;
        Opal::ScopePtr<Physics::SphereShape> ground_sphere(allocator, PHYSICS_CONST(1000.0));
        ground.shape = ground_sphere.Get();
        AddBody(ground);
        m_shapes.PushBack(std::move(ground_sphere));
    }

private:
    Opal::DynamicArray<Opal::ScopePtr<Physics::Shape>> m_shapes;
};

int main()
{
    SpheresSampleApp app;
    app.Run();
}