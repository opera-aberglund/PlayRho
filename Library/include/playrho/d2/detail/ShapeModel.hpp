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

#ifndef PLAYRHO_D2_DETAIL_SHAPEMODEL_HPP
#define PLAYRHO_D2_DETAIL_SHAPEMODEL_HPP

/// @file
/// @brief Definition of the @c ShapeModel class and related code.

#include "playrho/d2/ChainShapeConf.hpp"
#include "playrho/d2/DiskShapeConf.hpp"
#include "playrho/d2/EdgeShapeConf.hpp"
#include "playrho/d2/MultiShapeConf.hpp"
#include "playrho/d2/PolygonShapeConf.hpp"

#include <type_traits> // for std::enable_if_t, std::is_same_v
#include <utility> // for std::forward

#include <playrho/d2/detail/ShapeConcept.hpp>

namespace playrho::d2::detail {

/// @brief An "is valid shape type" trait.
/// @note This is the general false template type.
template <typename T, class = void>
struct IsValidShapeType : std::false_type {
};

/// @brief An "is valid shape type" trait.
/// @note This is the specialized true template type.
/// @note A shape can be constructed from or have its value set to any value whose type
///   <code>T</code> has at least the following function definitions available for it:
///   - <code>bool operator==(const T& lhs, const T& rhs) noexcept;</code>
///   - <code>ChildCounter GetChildCount(const T&) noexcept;</code>
///   - <code>DistanceProxy GetChild(const T&, ChildCounter index);</code>
///   - <code>MassData GetMassData(const T&) noexcept;</code>
///   - <code>NonNegative<Length> GetVertexRadius(const T&, ChildCounter idx);</code>
///   - <code>NonNegative<AreaDensity> GetDensity(const T&) noexcept;</code>
///   - <code>NonNegative<Real> GetFriction(const T&) noexcept;</code>
///   - <code>Real GetRestitution(const T&) noexcept;</code>
/// @see Shape
template <typename T>
struct IsValidShapeType<
    T,
    std::void_t<decltype(GetChildCount(std::declval<T>())), //
                decltype(GetChild(std::declval<T>(), std::declval<ChildCounter>())), //
                decltype(GetMassData(std::declval<T>())), //
                decltype(GetVertexRadius(std::declval<T>(), std::declval<ChildCounter>())), //
                decltype(GetDensity(std::declval<T>())), //
                decltype(GetFriction(std::declval<T>())), //
                decltype(GetRestitution(std::declval<T>())), //
                decltype(std::declval<T>() == std::declval<T>()), //
                decltype(std::declval<DecayedTypeIfNotSame<T, Shape>>()),
                decltype(std::is_constructible_v<DecayedTypeIfNotSame<T, Shape>, T>)>>
    : std::true_type {
};

/// @brief Return type for a SetFriction function taking an arbitrary type and a value.
/// @tparam T type to find function return type for.
template <class T>
using SetFrictionReturnType = decltype(SetFriction(std::declval<T&>(), std::declval<Real>()));

/// @brief Return type for a SetSensor function taking an arbitrary type and a value.
/// @tparam T type to find function return type for.
template <class T>
using SetSensorReturnType = decltype(SetSensor(std::declval<T&>(), std::declval<bool>()));

/// @brief Return type for a SetDensity function taking an arbitrary type and a value.
/// @tparam T type to find function return type for.
template <class T>
using SetDensityReturnType =
    decltype(SetDensity(std::declval<T&>(), std::declval<NonNegative<AreaDensity>>()));

/// @brief Return type for a SetRestitution function taking an arbitrary type and a value.
/// @tparam T type to find function return type for.
template <class T>
using SetRestitutionReturnType = decltype(SetRestitution(std::declval<T&>(), std::declval<Real>()));

/// @brief Return type for a SetFilter function taking an arbitrary type and a value.
/// @tparam T type to find function return type for.
template <class T>
using SetFilterReturnType = decltype(SetFilter(std::declval<T&>(), std::declval<Filter>()));

/// @brief Return type for a Translate function taking an arbitrary type and a value.
/// @tparam T type to find function return type for.
template <class T>
using TranslateReturnType = decltype(Translate(std::declval<T&>(), std::declval<Length2>()));

/// @brief Return type for a Scale function taking an arbitrary type and a value.
/// @tparam T type to find function return type for.
template <class T>
using ScaleReturnType = decltype(Scale(std::declval<T&>(), std::declval<Vec2>()));

/// @brief Return type for a Rotate function taking an arbitrary type and a value.
/// @tparam T type to find function return type for.
template <class T>
using RotateReturnType = decltype(Rotate(std::declval<T&>(), std::declval<Angle>()));

/// @brief Boolean value for whether the specified type is a valid shape type.
/// @see Shape.
template <class T>
constexpr bool IsValidShapeTypeV = IsValidShapeType<T>::value;

/// @brief Helper variable template on whether <code>SetFriction(T&, Real)</code> is found.
template <class T>
constexpr bool HasSetFrictionV = playrho::detail::is_detected_v<SetFrictionReturnType, T>;

/// @brief Helper variable template on whether <code>SetSensor(T&, bool)</code> is found.
template <class T>
constexpr bool HasSetSensorV = playrho::detail::is_detected_v<SetSensorReturnType, T>;

/// @brief Helper variable template on whether <code>SetDensity(T&, NonNegative<AreaDensity>)</code>
/// is found.
template <class T>
constexpr bool HasSetDensityV = playrho::detail::is_detected_v<SetDensityReturnType, T>;

/// @brief Helper variable template on whether <code>SetRestitution(T&, Real)</code> is found.
template <class T>
constexpr bool HasSetRestitutionV = playrho::detail::is_detected_v<SetRestitutionReturnType, T>;

/// @brief Helper variable template on whether <code>SetFilter(T&, Filter)</code> is found.
template <class T>
constexpr bool HasSetFilterV = playrho::detail::is_detected_v<SetFilterReturnType, T>;

/// @brief Helper variable template on whether <code>Translate(T&, Length2)</code> is found.
template <class T>
constexpr bool HasTranslateV = playrho::detail::is_detected_v<TranslateReturnType, T>;

/// @brief Helper variable template on whether <code>Scale(T&, Vec2)</code> is found.
template <class T>
constexpr bool HasScaleV = playrho::detail::is_detected_v<ScaleReturnType, T>;

/// @brief Helper variable template on whether <code>Rotate(T&, Angle)</code> is found.
template <class T>
constexpr bool HasRotateV = playrho::detail::is_detected_v<RotateReturnType, T>;

/// @brief Fallback friction setter that throws unless given the same value as current.
template <class T>
auto SetFriction(T& o, NonNegative<Real> value)
    -> std::enable_if_t<IsValidShapeTypeV<T> && !HasSetFrictionV<T>, void>
{
    if (GetFriction(o) != value) {
        throw InvalidArgument("SetFriction to non-equivalent value not supported");
    }
}

/// @brief Fallback sensor setter that throws unless given the same value as current.
template <class T>
auto SetSensor(T& o, bool value)
    -> std::enable_if_t<IsValidShapeTypeV<T> && !HasSetSensorV<T>, void>
{
    if (IsSensor(o) != value) {
        throw InvalidArgument("SetSensor to non-equivalent value not supported");
    }
}

/// @brief Fallback density setter that throws unless given the same value as current.
template <class T>
auto SetDensity(T& o, NonNegative<AreaDensity> value)
    -> std::enable_if_t<IsValidShapeTypeV<T> && !HasSetDensityV<T>, void>
{
    if (GetDensity(o) != value) {
        throw InvalidArgument("SetDensity to non-equivalent value not supported");
    }
}

/// @brief Fallback restitution setter that throws unless given the same value as current.
template <class T>
auto SetRestitution(T& o, Real value)
    -> std::enable_if_t<IsValidShapeTypeV<T> && !HasSetRestitutionV<T>, void>
{
    if (GetRestitution(o) != value) {
        throw InvalidArgument("SetRestitution to non-equivalent value not supported");
    }
}

/// @brief Fallback filter setter that throws unless given the same value as current.
template <class T>
auto SetFilter(T& o, Filter value)
    -> std::enable_if_t<IsValidShapeTypeV<T> && !HasSetFilterV<T>, void>
{
    if (GetFilter(o) != value) {
        throw InvalidArgument("SetFilter to non-equivalent filter not supported");
    }
}

/// @brief Fallback translate function that throws unless the given value has no effect.
template <class T>
auto Translate(T&, const Length2& value)
    -> std::enable_if_t<IsValidShapeTypeV<T> && !HasTranslateV<T>, void>
{
    if (Length2{} != value) {
        throw InvalidArgument("Translate non-zero amount not supported");
    }
}

/// @brief Fallback scale function that throws unless the given value has no effect.
template <class T>
auto Scale(T&, const Vec2& value) -> std::enable_if_t<IsValidShapeTypeV<T> && !HasScaleV<T>, void>
{
    if (Vec2{Real(1), Real(1)} != value) {
        throw InvalidArgument("Scale non-identity amount not supported");
    }
}

/// @brief Fallback rotate function that throws unless the given value has no effect.
template <class T>
auto Rotate(T&, const UnitVec& value)
    -> std::enable_if_t<IsValidShapeTypeV<T> && !HasRotateV<T>, void>
{
    if (UnitVec::GetRight() != value) {
        throw InvalidArgument("Rotate non-zero amount not supported");
    }
}

/// @brief Internal model configuration concept.
/// @note Provides an implementation for runtime polymorphism for shape configuration.
struct ShapeModel final {
    /// @brief Type alias for the type of the data held.
    using data_type = cista::offset::variant<DiskShapeConf, ChainShapeConf, EdgeShapeConf,
                                             PolygonShapeConf, MultiShapeConf>;

    /// @brief Initializing constructor.
    template <typename U, std::enable_if_t<!std::is_same_v<U, ShapeModel>, int> = 0>
    explicit ShapeModel(U&& arg) noexcept(std::is_nothrow_constructible_v<data_type, U>)
        : data{std::forward<U>(arg)}
    {
        // Intentionally empty.
    }

    auto cista_members()
    {
        return std::tie(data);
    }

    cista::offset::unique_ptr<ShapeModel> Clone_() const
    {
        return cista::offset::make_unique<ShapeModel>(data);
    }

    ChildCounter GetChildCount_() const noexcept
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            return GetChildCount(cista::get<DiskShapeConf>(data));
        }
        if (cista::holds_alternative<ChainShapeConf>(data)) {
            return GetChildCount(cista::get<ChainShapeConf>(data));
        }
        if (cista::holds_alternative<EdgeShapeConf>(data)) {
            return GetChildCount(cista::get<EdgeShapeConf>(data));
        }
        if (cista::holds_alternative<PolygonShapeConf>(data)) {
            return GetChildCount(cista::get<PolygonShapeConf>(data));
        }
        return GetChildCount(cista::get<MultiShapeConf>(data));
    }

    DistanceProxy GetChild_(ChildCounter index) const
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            return GetChild(cista::get<DiskShapeConf>(data), index);
        }
        if (cista::holds_alternative<ChainShapeConf>(data)) {
            return GetChild(cista::get<ChainShapeConf>(data), index);
        }
        if (cista::holds_alternative<EdgeShapeConf>(data)) {
            return GetChild(cista::get<EdgeShapeConf>(data), index);
        }
        if (cista::holds_alternative<PolygonShapeConf>(data)) {
            return GetChild(cista::get<PolygonShapeConf>(data), index);
        }
        return GetChild(cista::get<MultiShapeConf>(data), index);
    }

    MassData GetMassData_() const
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            return GetMassData(cista::get<DiskShapeConf>(data));
        }
        if (cista::holds_alternative<ChainShapeConf>(data)) {
            return GetMassData(cista::get<ChainShapeConf>(data));
        }
        if (cista::holds_alternative<EdgeShapeConf>(data)) {
            return GetMassData(cista::get<EdgeShapeConf>(data));
        }
        if (cista::holds_alternative<PolygonShapeConf>(data)) {
            return GetMassData(cista::get<PolygonShapeConf>(data));
        }
        return GetMassData(cista::get<MultiShapeConf>(data));
    }

    NonNegative<Length> GetVertexRadius_(ChildCounter idx) const
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            return GetVertexRadius(cista::get<DiskShapeConf>(data), idx);
        }
        if (cista::holds_alternative<ChainShapeConf>(data)) {
            return GetVertexRadius(cista::get<ChainShapeConf>(data), idx);
        }
        if (cista::holds_alternative<EdgeShapeConf>(data)) {
            return GetVertexRadius(cista::get<EdgeShapeConf>(data), idx);
        }
        if (cista::holds_alternative<PolygonShapeConf>(data)) {
            return GetVertexRadius(cista::get<PolygonShapeConf>(data), idx);
        }
        return GetVertexRadius(cista::get<MultiShapeConf>(data), idx);
    }

    void SetVertexRadius_(ChildCounter idx, NonNegative<Length> value)
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            SetVertexRadius(cista::get<DiskShapeConf>(data), idx, value);
        }
        else if (cista::holds_alternative<ChainShapeConf>(data)) {
            SetVertexRadius(cista::get<ChainShapeConf>(data), idx, value);
        }
        else if (cista::holds_alternative<EdgeShapeConf>(data)) {
            SetVertexRadius(cista::get<EdgeShapeConf>(data), idx, value);
        }
        else if (cista::holds_alternative<PolygonShapeConf>(data)) {
            SetVertexRadius(cista::get<PolygonShapeConf>(data), idx, value);
        }
        else {
            SetVertexRadius(cista::get<MultiShapeConf>(data), idx, value);
        }
    }

    NonNegative<AreaDensity> GetDensity_() const noexcept
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            return GetDensity(cista::get<DiskShapeConf>(data));
        }
        if (cista::holds_alternative<ChainShapeConf>(data)) {
            return GetDensity(cista::get<ChainShapeConf>(data));
        }
        if (cista::holds_alternative<EdgeShapeConf>(data)) {
            return GetDensity(cista::get<EdgeShapeConf>(data));
        }
        if (cista::holds_alternative<PolygonShapeConf>(data)) {
            return GetDensity(cista::get<PolygonShapeConf>(data));
        }
        return GetDensity(cista::get<MultiShapeConf>(data));
    }

    void SetDensity_(NonNegative<AreaDensity> value) noexcept
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            SetDensity(cista::get<DiskShapeConf>(data), value);
        }
        else if (cista::holds_alternative<ChainShapeConf>(data)) {
            SetDensity(cista::get<ChainShapeConf>(data), value);
        }
        else if (cista::holds_alternative<EdgeShapeConf>(data)) {
            SetDensity(cista::get<EdgeShapeConf>(data), value);
        }
        else if (cista::holds_alternative<PolygonShapeConf>(data)) {
            SetDensity(cista::get<PolygonShapeConf>(data), value);
        }
        else {
            SetDensity(cista::get<MultiShapeConf>(data), value);
        }
    }

    NonNegativeFF<Real> GetFriction_() const noexcept
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            return GetFriction(cista::get<DiskShapeConf>(data));
        }
        if (cista::holds_alternative<ChainShapeConf>(data)) {
            return GetFriction(cista::get<ChainShapeConf>(data));
        }
        if (cista::holds_alternative<EdgeShapeConf>(data)) {
            return GetFriction(cista::get<EdgeShapeConf>(data));
        }
        if (cista::holds_alternative<PolygonShapeConf>(data)) {
            return GetFriction(cista::get<PolygonShapeConf>(data));
        }
        return GetFriction(cista::get<MultiShapeConf>(data));
    }

    void SetFriction_(NonNegative<Real> value)
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            SetFriction(cista::get<DiskShapeConf>(data), value);
        }
        else if (cista::holds_alternative<ChainShapeConf>(data)) {
            SetFriction(cista::get<ChainShapeConf>(data), value);
        }
        else if (cista::holds_alternative<EdgeShapeConf>(data)) {
            SetFriction(cista::get<EdgeShapeConf>(data), value);
        }
        else if (cista::holds_alternative<PolygonShapeConf>(data)) {
            SetFriction(cista::get<PolygonShapeConf>(data), value);
        }
        else {
            SetFriction(cista::get<MultiShapeConf>(data), value);
        }
    }

    Real GetRestitution_() const noexcept
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            return GetRestitution(cista::get<DiskShapeConf>(data));
        }
        if (cista::holds_alternative<ChainShapeConf>(data)) {
            return GetRestitution(cista::get<ChainShapeConf>(data));
        }
        if (cista::holds_alternative<EdgeShapeConf>(data)) {
            return GetRestitution(cista::get<EdgeShapeConf>(data));
        }
        if (cista::holds_alternative<PolygonShapeConf>(data)) {
            return GetRestitution(cista::get<PolygonShapeConf>(data));
        }
        return GetRestitution(cista::get<MultiShapeConf>(data));
    }

    void SetRestitution_(Real value)
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            SetRestitution(cista::get<DiskShapeConf>(data), value);
        }
        else if (cista::holds_alternative<ChainShapeConf>(data)) {
            SetRestitution(cista::get<ChainShapeConf>(data), value);
        }
        else if (cista::holds_alternative<EdgeShapeConf>(data)) {
            SetRestitution(cista::get<EdgeShapeConf>(data), value);
        }
        else if (cista::holds_alternative<PolygonShapeConf>(data)) {
            SetRestitution(cista::get<PolygonShapeConf>(data), value);
        }
        else {
            SetRestitution(cista::get<MultiShapeConf>(data), value);
        }
    }

    Filter GetFilter_() const noexcept
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            return GetFilter(cista::get<DiskShapeConf>(data));
        }
        if (cista::holds_alternative<ChainShapeConf>(data)) {
            return GetFilter(cista::get<ChainShapeConf>(data));
        }
        if (cista::holds_alternative<EdgeShapeConf>(data)) {
            return GetFilter(cista::get<EdgeShapeConf>(data));
        }
        if (cista::holds_alternative<PolygonShapeConf>(data)) {
            return GetFilter(cista::get<PolygonShapeConf>(data));
        }
        return GetFilter(cista::get<MultiShapeConf>(data));
    }

    void SetFilter_(Filter value)
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            SetFilter(cista::get<DiskShapeConf>(data), value);
        }
        else if (cista::holds_alternative<ChainShapeConf>(data)) {
            SetFilter(cista::get<ChainShapeConf>(data), value);
        }
        else if (cista::holds_alternative<EdgeShapeConf>(data)) {
            SetFilter(cista::get<EdgeShapeConf>(data), value);
        }
        else if (cista::holds_alternative<PolygonShapeConf>(data)) {
            SetFilter(cista::get<PolygonShapeConf>(data), value);
        }
        else {
            SetFilter(cista::get<MultiShapeConf>(data), value);
        }
    }

    bool IsSensor_() const noexcept
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            return IsSensor(cista::get<DiskShapeConf>(data));
        }
        if (cista::holds_alternative<ChainShapeConf>(data)) {
            return IsSensor(cista::get<ChainShapeConf>(data));
        }
        if (cista::holds_alternative<EdgeShapeConf>(data)) {
            return IsSensor(cista::get<EdgeShapeConf>(data));
        }
        if (cista::holds_alternative<PolygonShapeConf>(data)) {
            return IsSensor(cista::get<PolygonShapeConf>(data));
        }
        return IsSensor(cista::get<MultiShapeConf>(data));
    }

    void SetSensor_(bool value)
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            SetSensor(cista::get<DiskShapeConf>(data), value);
        }
        else if (cista::holds_alternative<ChainShapeConf>(data)) {
            SetSensor(cista::get<ChainShapeConf>(data), value);
        }
        else if (cista::holds_alternative<EdgeShapeConf>(data)) {
            SetSensor(cista::get<EdgeShapeConf>(data), value);
        }
        else if (cista::holds_alternative<PolygonShapeConf>(data)) {
            SetSensor(cista::get<PolygonShapeConf>(data), value);
        }
        else {
            SetSensor(cista::get<MultiShapeConf>(data), value);
        }
    }

    void Translate_(const Length2& value)
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            Translate(cista::get<DiskShapeConf>(data), value);
        }
        else if (cista::holds_alternative<ChainShapeConf>(data)) {
            Translate(cista::get<ChainShapeConf>(data), value);
        }
        else if (cista::holds_alternative<EdgeShapeConf>(data)) {
            Translate(cista::get<EdgeShapeConf>(data), value);
        }
        else if (cista::holds_alternative<PolygonShapeConf>(data)) {
            Translate(cista::get<PolygonShapeConf>(data), value);
        }
        else {
            Translate(cista::get<MultiShapeConf>(data), value);
        }
    }

    void Scale_(const Vec2& value)
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            Scale(cista::get<DiskShapeConf>(data), value);
        }
        else if (cista::holds_alternative<ChainShapeConf>(data)) {
            Scale(cista::get<ChainShapeConf>(data), value);
        }
        else if (cista::holds_alternative<EdgeShapeConf>(data)) {
            Scale(cista::get<EdgeShapeConf>(data), value);
        }
        else if (cista::holds_alternative<PolygonShapeConf>(data)) {
            Scale(cista::get<PolygonShapeConf>(data), value);
        }
        else {
            Scale(cista::get<MultiShapeConf>(data), value);
        }
    }

    void Rotate_(const UnitVec& value)
    {
        if (cista::holds_alternative<DiskShapeConf>(data)) {
            Rotate(cista::get<DiskShapeConf>(data), value);
        }
        else if (cista::holds_alternative<ChainShapeConf>(data)) {
            Rotate(cista::get<ChainShapeConf>(data), value);
        }
        else if (cista::holds_alternative<EdgeShapeConf>(data)) {
            Rotate(cista::get<EdgeShapeConf>(data), value);
        }
        else if (cista::holds_alternative<PolygonShapeConf>(data)) {
            Rotate(cista::get<PolygonShapeConf>(data), value);
        }
        else {
            Rotate(cista::get<MultiShapeConf>(data), value);
        }
    }

    bool IsEqual_(const ShapeModel& other) const noexcept
    {
        // Would be preferable to do this without using any kind of RTTI system.
        // But how would that be done?
        return (GetType_() == other.GetType_()) && (data == other.data);
    }

    TypeID GetType_() const noexcept
    {
        return GetTypeID<data_type>();
    }

    const void* GetData_() const noexcept
    {
        // Note address of "data" not necessarily same as address of "this" since
        // base class is virtual.
        return &data;
    }

    data_type data; ///< Data.
};

} // namespace playrho::d2::detail

#endif // PLAYRHO_D2_DETAIL_SHAPEMODEL_HPP
