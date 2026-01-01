#include "shared/sample-app.hpp"

#include "physics/shapes/box-shape.hpp"

class SandboxSampleApp : public SampleApp
{
public:
    void SetupSimulation() override
    {
        {
            Physics::Body body;
            body.position = Physics::Vector3r(0, 0, 0);
            body.orientation = Physics::Quatr::Identity();
            body.linear_velocity = Physics::Vector3r(0, 0, 0);
            body.angular_velocity = Physics::Vector3r(0, 0, 0);
            body.inverse_mass = 0.0f;
            body.elasticity = 0.5f;
            body.friction = 0.5f;
            auto vertices = GetBoxVertices(Physics::Vector3r(100, 1, 50), Physics::Vector3r(0, 0, 0));
            Opal::ScopePtr<Physics::BoxShape> shape(Opal::GetDefaultAllocator());
            shape->Build(vertices);
            body.shape = shape.Get();
            m_shapes.PushBack(std::move(shape));
            AddBody(body);
        }
        {
            Physics::Body body;
            body.position = Physics::Vector3r(-50, 0, 0);
            body.orientation = Physics::Quatr::Identity();
            body.linear_velocity = Physics::Vector3r(0, 0, 0);
            body.angular_velocity = Physics::Vector3r(0, 0, 0);
            body.inverse_mass = 0.0f;
            body.elasticity = 0.5f;
            body.friction = 0.5f;
            auto vertices = GetBoxVertices(Physics::Vector3r(1, 5, 50), Physics::Vector3r(0, 0, 0));
            Opal::ScopePtr<Physics::BoxShape> shape(Opal::GetDefaultAllocator());
            shape->Build(vertices);
            body.shape = shape.Get();
            m_shapes.PushBack(std::move(shape));
            AddBody(body);
        }
        {
            Physics::Body body;
            body.position = Physics::Vector3r(50, 0, 0);
            body.orientation = Physics::Quatr::Identity();
            body.linear_velocity = Physics::Vector3r(0, 0, 0);
            body.angular_velocity = Physics::Vector3r(0, 0, 0);
            body.inverse_mass = 0.0f;
            body.elasticity = 0.5f;
            body.friction = 0.5f;
            auto vertices = GetBoxVertices(Physics::Vector3r(1, 5, 50), Physics::Vector3r(0, 0, 0));
            Opal::ScopePtr<Physics::BoxShape> shape(Opal::GetDefaultAllocator());
            shape->Build(vertices);
            body.shape = shape.Get();
            m_shapes.PushBack(std::move(shape));
            AddBody(body);
        }
        {
            Physics::Body body;
            body.position = Physics::Vector3r(0, 0, -25);
            body.orientation = Physics::Quatr::Identity();
            body.linear_velocity = Physics::Vector3r(0, 0, 0);
            body.angular_velocity = Physics::Vector3r(0, 0, 0);
            body.inverse_mass = 0.0f;
            body.elasticity = 0.5f;
            body.friction = 0.5f;
            auto vertices = GetBoxVertices(Physics::Vector3r(100, 5, 1), Physics::Vector3r(0, 0, 0));
            Opal::ScopePtr<Physics::BoxShape> shape(Opal::GetDefaultAllocator());
            shape->Build(vertices);
            body.shape = shape.Get();
            m_shapes.PushBack(std::move(shape));
            AddBody(body);
        }
        {
            Physics::Body body;
            body.position = Physics::Vector3r(0, 0, 25);
            body.orientation = Physics::Quatr::Identity();
            body.linear_velocity = Physics::Vector3r(0, 0, 0);
            body.angular_velocity = Physics::Vector3r(0, 0, 0);
            body.inverse_mass = 0.0f;
            body.elasticity = 0.5f;
            body.friction = 0.5f;
            auto vertices = GetBoxVertices(Physics::Vector3r(100, 5, 1), Physics::Vector3r(0, 0, 0));
            Opal::ScopePtr<Physics::BoxShape> shape(Opal::GetDefaultAllocator());
            shape->Build(vertices);
            body.shape = shape.Get();
            m_shapes.PushBack(std::move(shape));
            AddBody(body);
        }
    }

    void ResetSimulation() override
    {
        SampleApp::ResetSimulation();
        m_shapes.Clear();
        SetupSimulation();
    }

    Opal::DynamicArray<Physics::Vector3r> GetBoxVertices(Physics::Vector3r size, Physics::Vector3r offset = Physics::Vector3r::Zero())
    {
        Opal::DynamicArray vertices = {
            Physics::Vector3r(-size.x / 2, -size.y / 2, -size.z / 2), Physics::Vector3r(size.x / 2, -size.y / 2, -size.z / 2),
            Physics::Vector3r(-size.x / 2, size.y / 2, -size.z / 2),  Physics::Vector3r(size.x / 2, size.y / 2, -size.z / 2),
            Physics::Vector3r(-size.x / 2, -size.y / 2, size.z / 2),  Physics::Vector3r(size.x / 2, -size.y / 2, size.z / 2),
            Physics::Vector3r(-size.x / 2, size.y / 2, size.z / 2),   Physics::Vector3r(size.x / 2, size.y / 2, size.z / 2),
        };
        for (Physics::Vector3r& vertex : vertices)
        {
            vertex += offset;
        }
        return vertices;
    }

private:
    Opal::DynamicArray<Opal::ScopePtr<Physics::Shape>> m_shapes;
};

int main()
{
    SandboxSampleApp app;
    app.Run();
}