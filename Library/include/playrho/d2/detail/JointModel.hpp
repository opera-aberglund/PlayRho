/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef PLAYRHO_D2_DETAIL_JOINTMODEL_HPP
#define PLAYRHO_D2_DETAIL_JOINTMODEL_HPP

/// @file
/// @brief Definition of the internal @c JointModel class.

#include "playrho/d2/DistanceJointConf.hpp"
#include "playrho/d2/FrictionJointConf.hpp"
#include "playrho/d2/GearJointConf.hpp"
#include "playrho/d2/MotorJointConf.hpp"
#include "playrho/d2/PrismaticJointConf.hpp"
#include "playrho/d2/PulleyJointConf.hpp"
#include "playrho/d2/RevoluteJointConf.hpp"
#include "playrho/d2/RopeJointConf.hpp"
#include "playrho/d2/TargetJointConf.hpp"
#include "playrho/d2/WeldJointConf.hpp"
#include "playrho/d2/WheelJointConf.hpp"


#include <utility> // for std::move, std::forward
#include <type_traits> // for std::is_nothrow_constructible_v

#include <playrho/d2/detail/JointConcept.hpp>

namespace playrho::d2::detail {

/// @brief Internal joint model type.
/// @note Provides the implementation for runtime value polymorphism.
struct JointModel final : JointConcept {
    /// @brief Type alias for the type of the data held.
    using data_type =
        cista::offset::variant<RevoluteJointConf, PrismaticJointConf, DistanceJointConf,
                               PulleyJointConf, TargetJointConf, GearJointConf, WheelJointConf,
                               WeldJointConf, FrictionJointConf, MotorJointConf, RopeJointConf>;

    /// @brief Initializing constructor.
    template <typename U, std::enable_if_t<!std::is_same_v<U, JointModel>, int> = 0>
    explicit JointModel(U&& arg) noexcept(std::is_nothrow_constructible_v<data_type, U>)
        : data{std::forward<U>(arg)}
    {
        // Intentionally empty.
    }

    auto cista_members()
    {
        return std::tie(data);
    }

    std::unique_ptr<JointConcept> Clone_() const override
    {
        return std::make_unique<JointModel>(data);
    }

    cista::offset::unique_ptr<JointModel> Clone() const
    {
        return cista::offset::make_unique<JointModel>(data);
    }

    /// @copydoc JointConcept::GetType_
    TypeID GetType_() const noexcept override
    {
        return GetTypeID<data_type>();
    }

    /// @copydoc JointConcept::GetData_
    const void* GetData_() const noexcept override
    {
        // Note address of "data" not necessarily same as address of "this" since
        // base class is virtual.
        return &data;
    }

    /// @copydoc JointConcept::GetData_
    void* GetData_() noexcept override
    {
        // Note address of "data" not necessarily same as address of "this" since
        // base class is virtual.
        return &data;
    }

    bool IsEqual_(const JointConcept& other) const noexcept override
    {
        // Would be preferable to do this without using any kind of RTTI system.
        // But how would that be done?
        return (GetType_() == other.GetType_()) &&
               (data == *static_cast<const data_type*>(other.GetData_()));
    }

    /// @copydoc JointConcept::GetBodyA_
    BodyID GetBodyA_() const noexcept override
    {
        return data.apply([](const auto& arg) { return GetBodyA(arg); });
    }

    /// @copydoc JointConcept::GetBodyB_
    BodyID GetBodyB_() const noexcept override
    {
        return data.apply([](const auto& arg) { return GetBodyB(arg); });
    }

    /// @copydoc JointConcept::GetCollideConnected_
    bool GetCollideConnected_() const noexcept override
    {
        return data.apply([](const auto& arg) { return GetCollideConnected(arg); });
    }

    /// @copydoc JointConcept::ShiftOrigin_
    bool ShiftOrigin_(const Length2& value) noexcept override
    {
        return data.apply([&value](auto& arg) { return ShiftOrigin(arg, value); });
    }

    /// @copydoc JointConcept::InitVelocity_
    void InitVelocity_(const Span<BodyConstraint>& bodies, const playrho::StepConf& step,
                       const ConstraintSolverConf& conf) override
    {
        data.apply([&bodies, &step, &conf](auto& arg) { InitVelocity(arg, bodies, step, conf); });
    }

    /// @copydoc JointConcept::SolveVelocity_
    bool SolveVelocity_(const Span<BodyConstraint>& bodies, const playrho::StepConf& step) override
    {
        return data.apply([&bodies, &step](auto& arg) { return SolveVelocity(arg, bodies, step); });
    }

    /// @copydoc JointConcept::SolvePosition_
    bool SolvePosition_(const Span<BodyConstraint>& bodies,
                        const ConstraintSolverConf& conf) const override
    {
        return data.apply(
            [&bodies, &conf](const auto& arg) { return SolvePosition(arg, bodies, conf); });
    }

    data_type data; ///< Data.
};

} // namespace playrho::d2::detail

#endif // PLAYRHO_D2_DETAIL_JOINTMODEL_HPP
