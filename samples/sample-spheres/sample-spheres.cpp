#include "opal/rng.h"
#include "shared/sample-app.hpp"

class SpheresSampleApp : public SampleApp
{
public:
    void SetupSimulation() override
    {
        SetPhysicsUpdateInterval(0.016f);

        Opal::AllocatorBase* allocator = Opal::GetDefaultAllocator();

        for (Physics::i32 x = 0; x < 6; ++x)
        {
            for (Physics::i32 z = 0; z < 6; ++z)
            {
                const Physics::real radius = 0.5f;
                const Physics::real xx = static_cast<Physics::real>(x - 1) * radius * 1.5f;
                const Physics::real zz = static_cast<Physics::real>(z - 1) * radius * 1.5f;
                Physics::Body body;
                body.position = Physics::Vector3r{xx, 10, zz - 10};
                body.orientation = Physics::Quatr::Identity();
                body.linear_velocity = Physics::Vector3r::Zero();
                body.orientation = Physics::Quatr::Identity();
                body.inverse_mass = 1.0f;
                body.elasticity = 0.5f;
                body.friction = 0.5f;
                Opal::ScopePtr<Physics::SphereShape> sphere(allocator, radius);
                body.shape = sphere.Get();
                AddBody(body);
                m_shapes.PushBack(std::move(sphere));
            }
        }

        Opal::RNG rng;

        // Static Floor
        for (Physics::i32 x = 0; x < 3; ++x)
        {
            for (Physics::i32 z = 0; z < 3; ++z)
            {
                Physics::Body ground;
                const Physics::real radius = 80;
                ground.position = Physics::Vector3r{(x - 1) * radius * 0.25f, -radius + 2.0f, (z - 1) * radius * 0.25f - 10};
                ground.orientation = Physics::Quatr::FromAxisAngleDegrees(Physics::Vector3r{1, 0, 0}, 45.0f);
                ground.inverse_mass = 0.0f;
                ground.elasticity = 0.99f;
                ground.friction = 0.5f;
                Opal::ScopePtr<Physics::SphereShape> ground_sphere(allocator, radius);
                ground.shape = ground_sphere.Get();
                AddBody(ground);
                m_shapes.PushBack(std::move(ground_sphere));
            }
        }

        PauseSimulation();
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