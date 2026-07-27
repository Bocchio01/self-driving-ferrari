```cpp
#include "ferrari_vehicle/vehicle_model_factory.hpp"

#include <cstdio>
#include <variant>

using namespace vehicle_models::ackermann::kinematic_cartesian;

int main()
{
    // 1. Runtime selection via the factory.
    Params kc_params;
    kc_params.wheelbase = 0.26;
    kc_params.max_speed = 3.0;
    kc_params.max_steer = 0.5;
    VehicleModelVariant model_variant = VehicleModelFactory::create(ModelType::ACKERMANN_KINEMATIC_CARTESIAN, kc_params);

    // 2. Bind the concrete alternative once. std::get throws std::bad_variant_access
    //    if the runtime selection above didn't actually produce this type -- guard
    //    with std::holds_alternative in real code paths where that's possible.
    auto &model = std::get<Model>(model_variant);

    // 3. Hot loop: pure static dispatch on `model`, fixed-size Eigen, no allocation.
    State state{0.0, 0.0, 0.0};
    Command command{1.5, 0.2};

    auto x = Model::toVector(state);
    auto u = Model::toVector(command);
    const double dt = 0.02;

    StateJacobian A;
    ControlJacobian B;

    for (int i = 0; i < 5; ++i)
    {
        model.getLinearized(x, u, dt, A, B);
        x = model.step(x, u, dt);
    }

    const auto final_state = Model::toStruct(x);
    std::printf("x=%.4f y=%.4f yaw=%.4f\n", final_state.x, final_state.y, final_state.yaw);
    return 0;
}
```