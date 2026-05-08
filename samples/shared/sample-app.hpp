#pragma once

#include "opal/container/hash-map.h"
#include "opal/container/scope-ptr.h"

#include "physics/body.hpp"
#include "physics/core.hpp"
#include "physics/scene.hpp"

#include "rndr/application.hpp"
#include "rndr/canvas/context.hpp"
#include "rndr/canvas/draw-list.hpp"
#include "rndr/canvas/mesh.hpp"
#include "rndr/canvas/renderers/grid-renderer.hpp"
#include "rndr/canvas/renderers/pbr-renderer.hpp"

#include "shared/player-controller.hpp"

class SampleApp
{
public:
    SampleApp();
    virtual ~SampleApp();

    virtual void SetupSimulation() {}
    void Run();
    void ConditionallySimulateFrame(Physics::f32 delta_seconds);
    virtual void SimulateFrame(Physics::f32 delta_seconds) { m_scene.Update(delta_seconds); }
    void RenderFrame(Physics::f32 delta_seconds);
    void ToggleMovementControls();
    void PauseSimulation();
    void AdvanceSimulationFrame();
    virtual void ResetSimulation();
    void AddBody(Physics::Body body);
    void DrawBody(const Physics::Body& body);
    void SetPhysicsUpdateInterval(Physics::f32 interval) { m_physics_update_interval = interval; }
    void AddMesh(Physics::Shape* shape, Rndr::Canvas::Mesh mesh);

protected:
    Opal::ScopePtr<Rndr::Application> m_rndr_app;
    Opal::Ref<Rndr::GenericWindow> m_window;
    Rndr::Canvas::Context m_context;
    Opal::ScopePtr<PlayerController> m_player_controller;
    Opal::ScopePtr<Opal::LinearAllocator> m_scratch_allocator;
    Rndr::Canvas::GridRenderer m_grid_renderer;
    Rndr::Canvas::PbrRenderer m_pbr_renderer;
    Rndr::Canvas::PbrMaterialDesc m_default_material;

    Opal::HashMap<Physics::Shape*, Rndr::Canvas::Mesh> m_meshes;

    bool m_is_paused = false;
    bool m_advance_frame = false;

    Physics::f32 m_physics_update_interval = 0.033f;
    Physics::f32 m_time_until_physics_update = 0.033f;
    Physics::Scene m_scene;
};