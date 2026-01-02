#include "opal/container/in-place-array.h"

#include "physics/shapes/box-shape.hpp"
#include "physics/shapes/convex-shape.hpp"

#include "shared/sample-app.hpp"

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
        {
            auto diamond_vertices = GetDiamondVertices();
            Rndr::Mesh diamond_mesh = BuildDiamondMesh(diamond_vertices);
            Physics::Body body;
            body.position = Physics::Vector3r(0, 10, 0);
            body.orientation = Physics::Quatr::Identity();
            body.linear_velocity = Physics::Vector3r(0, 0, 0);
            body.angular_velocity = Physics::Vector3r(0, 0, 0);
            body.inverse_mass = 1.0f;
            body.elasticity = 0.5f;
            body.friction = 0.5f;
            Opal::ScopePtr<Physics::ConvexShape> shape(Opal::GetDefaultAllocator(), diamond_vertices);
            body.shape = shape.Get();
            AddMesh(body.shape, std::move(diamond_mesh));
            m_shapes.PushBack(std::move(shape));
            AddBody(body);
        }
        PauseSimulation();
    }

    void ResetSimulation() override
    {
        SampleApp::ResetSimulation();
        m_shapes.Clear();
        SetupSimulation();
    }

    static Opal::DynamicArray<Physics::Vector3r> GetBoxVertices(Physics::Vector3r size,
                                                                Physics::Vector3r offset = Physics::Vector3r::Zero())
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

    static Opal::DynamicArray<Physics::Vector3r> GetDiamondVertices()
    {
        Opal::InPlaceArray<Physics::Vector3r, 4 + 4> points;
        points[0] = Physics::Vector3r(0.1f, 0, -1);
        points[1] = Physics::Vector3r(1, 0, 0);
        points[2] = Physics::Vector3r(1, 0, 0.1f);
        points[3] = Physics::Vector3r(0.4f, 0, 0.4f);

        const Physics::Quatr quat_half =
            Physics::Quatr::FromAxisAngleRadians(Physics::Vector3r(0, 0, 1), 2.0f * 0.125f * 0.5f * Opal::k_pi_float);
        points[4] = Physics::Vector3r(0.8f, 0, 0.3f);
        points[4] = quat_half * points[4];
        points[5] = quat_half * points[1];
        points[6] = quat_half * points[2];

        const Physics::Quatr quat = Physics::Quatr::FromAxisAngleRadians(Physics::Vector3r(0, 0, 1), 2.0f * 0.125f * Opal::k_pi_float);
        Physics::i32 idx = 0;
        Opal::DynamicArray<Physics::Vector3r> out(7 * 8);
        for (Physics::i32 i = 0; i < 7; ++i)
        {
            out[idx] = points[i];
            ++idx;
        }

        Physics::Quatr quat_accumulator = Physics::Quatr::Identity();
        for (Physics::i32 i = 1; i < 8; ++i)
        {
            quat_accumulator = quat_accumulator * quat;
            for (Physics::i32 point_idx = 0; point_idx < 7; ++point_idx)
            {
                out[idx] = quat_accumulator * points[point_idx];
                ++idx;
            }
        }
        return out;
    }

    static Rndr::Mesh BuildDiamondMesh(Opal::ArrayView<Physics::Vector3r> vertices)
    {
        Opal::DynamicArray<Physics::Vector3r> hull_points;
        Opal::DynamicArray<Physics::Triangle> hull_triangles;
        Physics::BuildConvexHull(vertices, hull_points, hull_triangles);

        struct Vertex
        {
            Physics::Vector3r position;
            Physics::Vector3r normal;
            Physics::Vector2r uv;
        };

        Rndr::Mesh mesh;
        for (const Physics::Triangle& triangle : hull_triangles)
        {
            const Physics::Vector3r& a = hull_points[triangle.a];
            const Physics::Vector3r& b = hull_points[triangle.b];
            const Physics::Vector3r& c = hull_points[triangle.c];
            const Physics::Vector3r ab = b - a;
            const Physics::Vector3r ac = c - a;
            const Physics::Vector3r normal = Opal::Normalize(Opal::Cross(ab, ac));
            const Physics::Vector2r a_uv(0.5f + (std::atan2(a.z, a.x) / (2 * Opal::k_pi_float)),
                                         (0.5f - std::asin(a.y)) / Opal::k_pi_float);
            const Physics::Vector2r b_uv(0.5f + (std::atan2(b.z, b.x) / (2 * Opal::k_pi_float)),
                                         (0.5f - std::asin(b.y)) / Opal::k_pi_float);
            const Physics::Vector2r c_uv(0.5f + (std::atan2(c.z, c.x) / (2 * Opal::k_pi_float)),
                                         (0.5f - std::asin(c.y)) / Opal::k_pi_float);
            const Vertex vertex_a{.position = a, .normal = normal, .uv = a_uv};
            const Vertex vertex_b{.position = b, .normal = normal, .uv = b_uv};
            const Vertex vertex_c{.position = c, .normal = normal, .uv = c_uv};
            const Physics::i32 idx = static_cast<Physics::i32>(mesh.vertices.GetSize() / sizeof(Vertex));
            const Physics::i32 idx_1 = idx + 1;
            const Physics::i32 idx_2 = idx + 2;
            mesh.vertices.Append(Opal::AsBytes(vertex_a));
            mesh.vertices.Append(Opal::AsBytes(vertex_b));
            mesh.vertices.Append(Opal::AsBytes(vertex_c));
            mesh.indices.Append(Opal::AsBytes(idx));
            mesh.indices.Append(Opal::AsBytes(idx_2));
            mesh.indices.Append(Opal::AsBytes(idx_1));
        }
        mesh.index_count = static_cast<Physics::i32>(mesh.indices.GetSize() / sizeof(Physics::i32));
        mesh.vertex_count = static_cast<Physics::i32>(mesh.vertices.GetSize() / sizeof(Vertex));
        mesh.index_size = 4;
        mesh.vertex_size = sizeof(Vertex);
        mesh.name = "Diamond";
        return mesh;
    }

private:
    Opal::DynamicArray<Opal::ScopePtr<Physics::Shape>> m_shapes;
};

int main()
{
    SandboxSampleApp app;
    app.Run();
}