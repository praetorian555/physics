#include "shared/sample-app.hpp"

class SpheresSampleApp : public SampleApp
{
public:
    void SetupSimulation() override
    {
        Physics::Body body;
        body.position = Physics::Vector3r{0, 0, -10};
        body.orientation = Physics::Quatr::Identity();
        body.shape = Opal::New<Physics::SphereShape>(Opal::GetDefaultAllocator(), 1.0f);
        AddBody(body);
    }
};

int main()
{
    SpheresSampleApp app;
    app.Run();
}