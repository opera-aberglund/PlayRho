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

#include <playrho/d2/Body.hpp>

#include "gtest/gtest.h"

using namespace playrho;
using namespace playrho::d2;

TEST(Body, DefaultConstruction)
{
    EXPECT_EQ(Body().GetType(), BodyType::Static);
    EXPECT_TRUE(Body().IsEnabled());
    EXPECT_FALSE(Body().IsAwake());
    EXPECT_FALSE(Body().IsSpeedable());
    EXPECT_FALSE(Body().IsDestroyed());
    EXPECT_TRUE(IsEnabled(Body()));
    EXPECT_FALSE(IsAwake(Body()));
    EXPECT_FALSE(IsSpeedable(Body()));
    EXPECT_EQ(Body().GetLinearDamping(), Body::DefaultLinearDamping);
    EXPECT_EQ(Body().GetAngularDamping(), Body::DefaultAngularDamping);
}

TEST(Body, StaticTypeConstruction)
{
    const auto body = Body(BodyConf().Use(BodyType::Static));
    EXPECT_FALSE(body.IsDestroyed());
    EXPECT_TRUE(body.IsImpenetrable());
    EXPECT_FALSE(body.IsSpeedable());
    EXPECT_FALSE(body.IsAccelerable());
    EXPECT_TRUE(body.IsSleepingAllowed());
    EXPECT_FALSE(body.IsAwake());
    EXPECT_FALSE(body.IsFixedRotation());
    EXPECT_FALSE(body.IsMassDataDirty());
    EXPECT_TRUE(body.IsEnabled());
}

TEST(Body, KinematicTypeConstruction)
{
    const auto body = Body(BodyConf().Use(BodyType::Kinematic));
    EXPECT_FALSE(body.IsDestroyed());
    EXPECT_TRUE(body.IsImpenetrable());
    EXPECT_TRUE(body.IsSpeedable());
    EXPECT_FALSE(body.IsAccelerable());
    EXPECT_TRUE(body.IsSleepingAllowed());
    EXPECT_TRUE(body.IsAwake());
    EXPECT_FALSE(body.IsFixedRotation());
    EXPECT_FALSE(body.IsMassDataDirty());
    EXPECT_TRUE(body.IsEnabled());
}

TEST(Body, DynamicTypeConstruction)
{
    const auto body = Body(BodyConf().Use(BodyType::Dynamic));
    EXPECT_FALSE(body.IsDestroyed());
    EXPECT_FALSE(body.IsImpenetrable());
    EXPECT_TRUE(body.IsSpeedable());
    EXPECT_TRUE(body.IsAccelerable());
    EXPECT_TRUE(body.IsSleepingAllowed());
    EXPECT_TRUE(body.IsAwake());
    EXPECT_FALSE(body.IsFixedRotation());
    EXPECT_FALSE(body.IsMassDataDirty());
    EXPECT_TRUE(body.IsEnabled());
}

TEST(Body, UseFixedRotationConstruction)
{
    EXPECT_TRUE(Body(BodyConf().UseFixedRotation(true)).IsFixedRotation());
    EXPECT_FALSE(Body(BodyConf().UseFixedRotation(false)).IsFixedRotation());
}

TEST(Body, UseAwakeConstruction)
{
    EXPECT_FALSE(Body(BodyConf().Use(BodyType::Static).UseAwake(true)).IsAwake());
    EXPECT_FALSE(Body(BodyConf().Use(BodyType::Static).UseAwake(false)).IsAwake());
    EXPECT_TRUE(Body(BodyConf().Use(BodyType::Kinematic).UseAwake(true)).IsAwake());
    EXPECT_FALSE(Body(BodyConf().Use(BodyType::Kinematic).UseAwake(false)).IsAwake());
    EXPECT_TRUE(Body(BodyConf().Use(BodyType::Dynamic).UseAwake(true)).IsAwake());
    EXPECT_FALSE(Body(BodyConf().Use(BodyType::Dynamic).UseAwake(false)).IsAwake());
    EXPECT_TRUE(Body(BodyConf().Use(BodyType::Dynamic).UseAllowSleep(false).UseAwake(false)).IsAwake());
}

TEST(Body, UseAllowSleepConstruction)
{
    EXPECT_TRUE(Body(BodyConf().Use(BodyType::Static).UseAllowSleep(true)).IsSleepingAllowed());
    EXPECT_FALSE(Body(BodyConf().Use(BodyType::Static).UseAllowSleep(false)).IsSleepingAllowed());
    EXPECT_TRUE(Body(BodyConf().Use(BodyType::Kinematic).UseAllowSleep(true)).IsSleepingAllowed());
    EXPECT_FALSE(Body(BodyConf().Use(BodyType::Kinematic).UseAllowSleep(false)).IsSleepingAllowed());
    EXPECT_TRUE(Body(BodyConf().Use(BodyType::Dynamic).UseAllowSleep(true)).IsSleepingAllowed());
    EXPECT_FALSE(Body(BodyConf().Use(BodyType::Dynamic).UseAllowSleep(false)).IsSleepingAllowed());
}

TEST(Body, ShapeOnConstruction)
{
    const auto shapeId = ShapeID(1u);
    ASSERT_TRUE(!empty(Body(BodyConf{}.Use(shapeId)).GetShapes()));
    ASSERT_EQ(size(Body(BodyConf{}.Use(shapeId)).GetShapes()), 1u);
    EXPECT_EQ(Body(BodyConf{}.Use(shapeId)).GetShapes()[0], shapeId);
}

TEST(Body, LinearDampingOnConstruction)
{
    EXPECT_EQ(Body(BodyConf{}.UseLinearDamping(0_Hz)).GetLinearDamping(), 0_Hz);
    EXPECT_EQ(Body(BodyConf{}.UseLinearDamping(20_Hz)).GetLinearDamping(), 20_Hz);
    EXPECT_EQ(Body(BodyConf{}.UseLinearDamping(30_Hz)).GetLinearDamping(), 30_Hz);
}

TEST(Body, AngularDampingOnConstruction)
{
    EXPECT_EQ(Body(BodyConf{}.UseAngularDamping(0_Hz)).GetAngularDamping(), 0_Hz);
    EXPECT_EQ(Body(BodyConf{}.UseAngularDamping(20_Hz)).GetAngularDamping(), 20_Hz);
    EXPECT_EQ(Body(BodyConf{}.UseAngularDamping(30_Hz)).GetAngularDamping(), 30_Hz);
}

TEST(Body, InvMassOnConstruction)
{
    EXPECT_EQ(Body(BodyConf{}.Use(BodyType::Dynamic)).GetInvMass(), Real(1) / 1_kg);
    EXPECT_EQ(Body(BodyConf{}.Use(BodyType::Kinematic)).GetInvMass(), Real(0) / 1_kg);
    EXPECT_EQ(Body(BodyConf{}.Use(BodyType::Static)).GetInvMass(), Real(0) / 1_kg);
}

TEST(Body, TransformationOnConstruction)
{
    EXPECT_EQ(
        Body(BodyConf{}.UseLocation(Length2{10_m, 12_m}).UseAngle(90_deg)).GetTransformation(),
        ::playrho::d2::GetTransformation(
            BodyConf{}.UseLocation(Length2{10_m, 12_m}).UseAngle(90_deg)));
    EXPECT_EQ(
        Body(BodyConf{}.UseLocation(Length2{4_m, -3_m}).UseAngle(-32_deg)).GetTransformation(),
        ::playrho::d2::GetTransformation(
            BodyConf{}.UseLocation(Length2{4_m, -3_m}).UseAngle(-32_deg)));
}

TEST(Body, VelocityOnConstruction)
{
    auto body = Body{};
    body.SetVelocity(Velocity{LinearVelocity2{1_mps, 2_mps}, 3_rpm});
    EXPECT_EQ(
        Body(BodyConf{}.Use(Velocity{LinearVelocity2{1_mps, 2_mps}, 3_rpm})).GetVelocity().linear,
        body.GetVelocity().linear);
    EXPECT_EQ(
        Body(BodyConf{}.Use(Velocity{LinearVelocity2{1_mps, 2_mps}, 3_rpm})).GetVelocity().angular,
        body.GetVelocity().angular);
}

TEST(Body, AccelerationOnConstruction)
{
    auto body = Body{};
    body.SetAcceleration(LinearAcceleration2{2_mps2, 3_mps2}, Real(4) * RadianPerSquareSecond);
    EXPECT_EQ(Body(BodyConf{}
                       .UseLinearAcceleration(LinearAcceleration2{2_mps2, 3_mps2})
                       .UseAngularAcceleration(Real(4) * RadianPerSquareSecond))
                  .GetLinearAcceleration(),
              body.GetLinearAcceleration());
    EXPECT_EQ(Body(BodyConf{}
                       .UseLinearAcceleration(LinearAcceleration2{2_mps2, 3_mps2})
                       .UseAngularAcceleration(Real(4) * RadianPerSquareSecond))
                  .GetAngularAcceleration(),
              body.GetAngularAcceleration());
}

TEST(Body, SetUnsetDestroyed)
{
    auto body = Body();
    ASSERT_FALSE(body.IsDestroyed());
    body.SetDestroyed();
    EXPECT_TRUE(body.IsDestroyed());
    body.UnsetDestroyed();
    EXPECT_FALSE(body.IsDestroyed());
    SetDestroyed(body);
    EXPECT_TRUE(IsDestroyed(body));
    UnsetDestroyed(body);
    EXPECT_FALSE(IsDestroyed(body));
}

TEST(Body, EqualsOperator)
{
    EXPECT_TRUE(Body() == Body());
    {
        auto body = Body{};
        EXPECT_TRUE(body == Body());
    }
    {
        auto bodyA = Body();
        auto bodyB = Body();
        bodyB.SetDestroyed();
        EXPECT_FALSE(bodyA == bodyB);
        bodyA.SetDestroyed();
        EXPECT_TRUE(bodyA == bodyB);
        bodyB.UnsetDestroyed();
        EXPECT_FALSE(bodyA == bodyB);
        bodyA.UnsetDestroyed();
        EXPECT_TRUE(bodyA == bodyB);
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_TRUE(body1 == body2);
    }
    {
        auto body = Body{};
        SetTransformation(body, Transformation{Length2{2_m, 0_m}, UnitVec{}});
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        body.SetSweep(Sweep{Position{Length2{}, 2_deg}});
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        body.SetType(BodyType::Kinematic);
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        body.SetType(BodyType::Kinematic);
        body.JustSetVelocity(Velocity{LinearVelocity2{}, 2_rpm});
        EXPECT_FALSE(body == Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        body1.SetAcceleration(LinearAcceleration2{}, Real(2) * RadianPerSquareSecond);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_FALSE(body1 == body2);
    }
    {
        auto body = Body{};
        SetMass(body, 3.2_kg);
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        body.SetInvMassData(body.GetInvMass(), (Real(2) * SquareRadian) / (2_m2 * 1.2_kg));
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        SetLinearDamping(body, 2_Hz);
        EXPECT_FALSE(body == Body());
    }
    {
        auto body = Body{};
        SetAngularDamping(body, 2_Hz);
        EXPECT_FALSE(body == Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        body1.SetUnderActiveTime(2_s);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_FALSE(body1 == body2);
    }
    {
        auto bodyA = Body{};
        auto bodyB = Body{};
        bodyA.Attach(ShapeID(1));
        EXPECT_FALSE(bodyA == bodyB);
        bodyB.Attach(ShapeID(1));
        EXPECT_TRUE(bodyA == bodyB);
    }
    {
        auto bodyA = Body{};
        bodyA.SetType(BodyType::Dynamic);
        ASSERT_TRUE(IsSleepingAllowed(bodyA));
        auto bodyB = Body{};
        bodyB.SetType(BodyType::Dynamic);
        ASSERT_TRUE(IsSleepingAllowed(bodyA));
        bodyA.SetSleepingAllowed(false);
        ASSERT_FALSE(IsSleepingAllowed(bodyA));
        EXPECT_FALSE(bodyA == bodyB);
    }
}

TEST(Body, NotEqualsOperator)
{
    EXPECT_FALSE(Body() != Body());
    {
        auto body = Body{};
        EXPECT_FALSE(body != Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_FALSE(body1 != body2);
    }
    {
        auto body = Body{};
        SetTransformation(body, Transformation{Length2{2_m, 0_m}, UnitVec{}});
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        body.SetSweep(Sweep{Position{Length2{}, 2_deg}});
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        body.SetType(BodyType::Kinematic);
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        body.SetType(BodyType::Kinematic);
        body.JustSetVelocity(Velocity{LinearVelocity2{}, 2_rpm});
        EXPECT_TRUE(body != Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        body1.SetAcceleration(LinearAcceleration2{}, Real(2) * RadianPerSquareSecond);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_TRUE(body1 != body2);
    }
    {
        auto body = Body{};
        SetMass(body, 3.2_kg);
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        body.SetInvMassData(body.GetInvMass(), (Real(2) * SquareRadian) / (2_m2 * 1.2_kg));
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        SetLinearDamping(body, 2_Hz);
        EXPECT_TRUE(body != Body());
    }
    {
        auto body = Body{};
        SetAngularDamping(body, 2_Hz);
        EXPECT_TRUE(body != Body());
    }
    {
        auto body1 = Body{};
        body1.SetType(BodyType::Dynamic);
        body1.SetUnderActiveTime(2_s);
        auto body2 = Body{};
        body2.SetType(BodyType::Dynamic);
        EXPECT_TRUE(body1 != body2);
    }
}
