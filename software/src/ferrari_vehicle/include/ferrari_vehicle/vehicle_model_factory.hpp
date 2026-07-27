#pragma once

#include "ferrari_vehicle/ackermann_models/dynamic_cartesian.hpp"
#include "ferrari_vehicle/ackermann_models/kinematic_cartesian.hpp"
#include "ferrari_vehicle/ackermann_models/kinematic_frenet.hpp"

#include <utility>
#include <variant>

namespace vehicle_models
{

    /**
     * @brief Enum class for specifying the type of vehicle model to create.
     */
    enum class ModelType
    {
        ACKERMANN_KINEMATIC_CARTESIAN,
        ACKERMANN_KINEMATIC_FRENET,
        ACKERMANN_DYNAMIC_CARTESIAN
    };

    /**
     * @brief Variant type to hold different vehicle model types.
     */
    using VehicleModelVariant = std::variant<ackermann::kinematic_cartesian::Model,
                                             ackermann::kinematic_frenet::Model,
                                             ackermann::dynamic_cartesian::Model>;

    /**
     * @brief Factory class for creating vehicle models of different types.
     *
     * This class provides static methods to create vehicle models based on the specified type and parameters.
     * It uses std::variant to hold different types of vehicle models, allowing for runtime selection
     * of the model type while maintaining static dispatch for performance.
     *
     * For performance, it's recommended to directly call the constructor of the desired model or use the
     * factory to create a variant and then extract the concrete model type for use in the hot loop.
     */
    class VehicleModelFactory
    {
    public:
        /**
         * @brief Create a vehicle model of the specified type with the given parameters.
         * @param type The type of vehicle model to create
         * @param params The parameters for the vehicle model
         * @return A variant containing the created vehicle model
         */
        static VehicleModelVariant create(ModelType type, const ackermann::kinematic_cartesian::Params &params)
        {
            (void)type; // overload selects the concrete model; type is accepted for call-site clarity
            return VehicleModelVariant{ackermann::kinematic_cartesian::Model(params)};
        }

        static VehicleModelVariant create(ModelType type, const ackermann::kinematic_frenet::Params &params)
        {
            (void)type;
            return VehicleModelVariant{ackermann::kinematic_frenet::Model(params)};
        }

        static VehicleModelVariant create(ModelType type, const ackermann::dynamic_cartesian::Params &params)
        {
            (void)type;
            return VehicleModelVariant{ackermann::dynamic_cartesian::Model(params)};
        }
    };

    /**
     * @brief Apply a generic visitor to the current vehicle model.
     * @param model The vehicle model variant
     * @param visitor The visitor to apply
     * @return The result of the visit
     */
    template <typename Visitor>
    decltype(auto) visitModel(VehicleModelVariant &model, Visitor &&visitor)
    {
        return std::visit(std::forward<Visitor>(visitor), model);
    }

    template <typename Visitor>
    decltype(auto) visitModel(const VehicleModelVariant &model, Visitor &&visitor)
    {
        return std::visit(std::forward<Visitor>(visitor), model);
    }

} // namespace vehicle_models
