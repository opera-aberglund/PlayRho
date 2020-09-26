/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"
#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(RevoluteJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(RevoluteJoint), std::size_t(140));
#else
            EXPECT_EQ(sizeof(RevoluteJoint), std::size_t(144));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(RevoluteJoint), std::size_t(280)); break;
        case 16: EXPECT_EQ(sizeof(RevoluteJoint), std::size_t(528)); break;
        default: FAIL(); break;
    }
}

TEST(RevoluteJoint, Construction)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();

    auto jd = RevoluteJointConf{};

    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.collideConnected = true;
    jd.userData = reinterpret_cast<void*>(0x011);

    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    jd.enableLimit = true;
    jd.enableMotor = true;
    jd.motorSpeed = Real{4.4f} * RadianPerSecond;
    jd.maxMotorTorque = 1_Nm;
    jd.lowerAngle = 33_deg;
    jd.upperAngle = 40_deg;
    jd.referenceAngle = 45_deg;
    
    auto joint = RevoluteJoint{jd};

    EXPECT_EQ(GetType(joint), jd.type);
    EXPECT_EQ(joint.GetBodyA(), jd.bodyA);
    EXPECT_EQ(joint.GetBodyB(), jd.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), jd.collideConnected);
    EXPECT_EQ(joint.GetUserData(), jd.userData);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});
    EXPECT_EQ(joint.GetLimitState(), Joint::e_inactiveLimit);

    EXPECT_EQ(joint.GetLocalAnchorA(), jd.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), jd.localAnchorB);
    EXPECT_EQ(joint.GetLowerLimit(), jd.lowerAngle);
    EXPECT_EQ(joint.GetUpperLimit(), jd.upperAngle);
    EXPECT_EQ(joint.GetMotorSpeed(), jd.motorSpeed);
    EXPECT_EQ(joint.GetReferenceAngle(), jd.referenceAngle);
    EXPECT_EQ(joint.IsMotorEnabled(), jd.enableMotor);
    EXPECT_EQ(joint.GetMaxMotorTorque(), jd.maxMotorTorque);
    EXPECT_EQ(joint.IsLimitEnabled(), jd.enableLimit);
    EXPECT_EQ(joint.GetMotorImpulse(), AngularMomentum{0});

    EXPECT_EQ(GetAngularVelocity(world, joint), 0 * RadianPerSecond);

    const auto id = world.CreateJoint(jd);
    EXPECT_EQ(GetAnchorA(world, id), Length2(4_m, 5_m));
    EXPECT_EQ(GetAnchorB(world, id), Length2(6_m, 7_m));

    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Revolute);
    
    EXPECT_EQ(GetMotorTorque(joint, 1_Hz), 0 * NewtonMeter);
}

TEST(RevoluteJoint, EnableMotor)
{
    World world;
    const auto b0 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    ASSERT_EQ(GetVelocity(world, b0), Velocity{});
    ASSERT_EQ(GetVelocity(world, b1), Velocity{});
    
    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);

    auto joint = RevoluteJoint{jd};
    ASSERT_FALSE(joint.IsLimitEnabled());
    EXPECT_EQ(joint.GetLimitState(), Joint::e_inactiveLimit);
    EXPECT_FALSE(joint.IsMotorEnabled());
    joint.EnableMotor(false);
    EXPECT_FALSE(joint.IsMotorEnabled());
    joint.EnableMotor(true);
    EXPECT_TRUE(joint.IsMotorEnabled());
}

#if 0
TEST(RevoluteJoint, EnableMotorInWorld)
{
    World world;
    const auto b0 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    ASSERT_EQ(GetVelocity(world, b0), Velocity{});
    ASSERT_EQ(GetVelocity(world, b1), Velocity{});
    
    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);

    const auto id = world.CreateJoint(jd);
    ASSERT_NE(id, InvalidJointID);
    ASSERT_EQ(GetVelocity(world, b0), Velocity{});
    ASSERT_EQ(GetVelocity(world, b1), Velocity{});
    EXPECT_FALSE(IsMotorEnabled(world, id));
    EnableMotor(world, id, false);
    EXPECT_FALSE(IsMotorEnabled(world, id));
    EnableMotor(world, id, true);
    EXPECT_TRUE(IsMotorEnabled(world, id));

    const auto newValue = 5_Nm;
    ASSERT_NE(GetMaxMotorTorque(world, id), newValue);
    EXPECT_EQ(GetMaxMotorTorque(world, id), jd.maxMotorTorque);
    SetMaxMotorTorque(world, id, newValue);
    EXPECT_EQ(GetMaxMotorTorque(world, id), newValue);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));
    
    const auto shape = Shape(DiskShapeConf{}.UseRadius(1_m).UseDensity(1_kgpm2));
    world.CreateFixture(b0, shape);
    world.CreateFixture(b1, shape);
    ASSERT_NE(GetInvRotInertia(world, b0), InvRotInertia(0));
    ASSERT_NE(GetInvRotInertia(world, b1), InvRotInertia(0));
    
    auto stepConf = StepConf{};
    world.Step(stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));
    stepConf.doWarmStart = false;
    world.Step(stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));
    EXPECT_NE(GetVelocity(world, b0), Velocity{});
    EXPECT_NE(GetVelocity(world, b1), Velocity{});

    joint.EnableLimit(true);
    ASSERT_TRUE(joint.IsLimitEnabled());
    
    joint.SetLimits(-45_deg, -5_deg);

    stepConf.doWarmStart = true;
    world.Step(stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));
    EXPECT_EQ(GetAngularReaction(world, id), AngularMomentum(0));
    EXPECT_EQ(GetLimitState(world, id), Joint::e_atUpperLimit);
    EXPECT_NE(GetVelocity(world, b0), Velocity{});
    EXPECT_NE(GetVelocity(world, b1), Velocity{});

    joint->SetLimits(+55_deg, +95_deg);
    
    stepConf.doWarmStart = true;
    world.Step(stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));
    EXPECT_EQ(GetAngularReaction(world, id), AngularMomentum(0));
    EXPECT_EQ(GetLimitState(world, id), Joint::e_atLowerLimit);
    
    EXPECT_NE(GetVelocity(world, b0), Velocity{});
    EXPECT_NE(GetVelocity(world, b1), Velocity{});
}
#endif

TEST(RevoluteJoint, MotorSpeed)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    const auto newValue = Real(5) * RadianPerSecond;
    auto joint = RevoluteJoint{jd};
    ASSERT_NE(joint.GetMotorSpeed(), newValue);
    EXPECT_EQ(joint.GetMotorSpeed(), jd.motorSpeed);
    joint.SetMotorSpeed(newValue);
    EXPECT_EQ(joint.GetMotorSpeed(), newValue);
}

#if 0
TEST(RevoluteJoint, EnableLimit)
{
    auto world = World{};
    const auto b0 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_EQ(GetInvRotInertia(world, b0), InvRotInertia(0));
    ASSERT_EQ(GetInvRotInertia(world, b1), InvRotInertia(0));

    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    jd.enableLimit = false;

    auto joint = RevoluteJoint{jd};
    ASSERT_EQ(joint.GetLimitState(), Joint::e_inactiveLimit);
    ASSERT_FALSE(joint.IsLimitEnabled());
    joint.EnableLimit(false);
    EXPECT_FALSE(joint.IsLimitEnabled());
    joint.EnableLimit(true);
    EXPECT_TRUE(joint.IsLimitEnabled());

    const auto id = world.CreateJoint(jd);
    ASSERT_NE(id, InvalidJointID);

    auto stepConf = StepConf{};
    world.Step(stepConf);
    EXPECT_TRUE(joint->IsLimitEnabled());
    EXPECT_EQ(joint->GetLimitState(), Joint::e_inactiveLimit); // since b0 & b1 inv rot inertia 0
    
    const auto shape = Shape(DiskShapeConf{}.UseRadius(1_m).UseDensity(1_kgpm2));
    world.CreateFixture(b0, shape);
    world.CreateFixture(b1, shape);
    ASSERT_NE(GetInvRotInertia(world, b0), InvRotInertia(0));
    ASSERT_NE(GetInvRotInertia(world, b1), InvRotInertia(0));
    
    world.Step(stepConf);
    EXPECT_TRUE(joint->IsLimitEnabled());
    EXPECT_EQ(joint->GetLimitState(), Joint::e_equalLimits);
    
    joint->SetLimits(-45_deg, +45_deg);
    ASSERT_TRUE(joint->IsLimitEnabled());
    ASSERT_EQ(joint->GetLimitState(), Joint::e_equalLimits);
    world.Step(stepConf);
    
    EXPECT_TRUE(joint->IsLimitEnabled());
    EXPECT_EQ(joint->GetLimitState(), Joint::e_inactiveLimit);
    
    EXPECT_EQ(joint->GetMotorImpulse(), AngularMomentum(0));
}
#endif

TEST(RevoluteJoint, SetLimits)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    const auto upperValue = +5_deg;
    const auto lowerValue = -8_deg;
    auto joint = RevoluteJoint{jd};
    ASSERT_NE(joint.GetUpperLimit(), upperValue);
    ASSERT_NE(joint.GetLowerLimit(), lowerValue);
    joint.SetLimits(lowerValue, upperValue);
    EXPECT_EQ(joint.GetUpperLimit(), upperValue);
    EXPECT_EQ(joint.GetLowerLimit(), lowerValue);
}

#if 0
TEST(RevoluteJoint, MaxMotorTorque)
{
    World world;
    const auto b0 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    
    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    const auto newValue = 5_Nm;
    const auto joint = static_cast<RevoluteJoint*>(world.CreateJoint(jd));
    ASSERT_NE(joint, nullptr);

    ASSERT_NE(joint->GetMaxMotorTorque(), newValue);
    EXPECT_EQ(joint->GetMaxMotorTorque(), jd.maxMotorTorque);
    joint->SetMaxMotorTorque(newValue);
    EXPECT_EQ(joint->GetMaxMotorTorque(), newValue);
    EXPECT_EQ(joint->GetMotorImpulse(), AngularMomentum(0));
    
    const auto shape = Shape(DiskShapeConf{}.UseRadius(1_m).UseDensity(1_kgpm2));
    world.CreateFixture(b0, shape);
    world.CreateFixture(b1, shape);
    ASSERT_NE(GetInvRotInertia(world, b0), InvRotInertia(0));
    ASSERT_NE(GetInvRotInertia(world, b1), InvRotInertia(0));
    
    auto stepConf = StepConf{};
    world.Step(stepConf);
    EXPECT_EQ(joint->GetMotorImpulse(), AngularMomentum(0));
    stepConf.doWarmStart = false;
    world.Step(stepConf);
    EXPECT_EQ(joint->GetMotorImpulse(), AngularMomentum(0));
}
#endif

TEST(RevoluteJoint, MovesDynamicCircles)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    World world;
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1).UseLinearAcceleration(EarthlyGravity));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2).UseLinearAcceleration(EarthlyGravity));
    world.CreateFixture(b1, circle);
    world.CreateFixture(b2, circle);
    auto jd = RevoluteJointConf{};
    jd.bodyA = b1;
    jd.bodyB = b2;
    world.CreateJoint(jd);

    auto step = StepConf{};
    step.SetTime(1_s);
    step.maxTranslation = Meter * Real(4);
    world.Step(step);

    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), 0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), -4, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), 0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), -4, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

#if 0
TEST(RevoluteJoint, LimitEnabledDynamicCircles)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m).UseDensity(1_kgpm2)};

    World world;
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1).UseLinearAcceleration(EarthlyGravity));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2).UseLinearAcceleration(EarthlyGravity));
    world.CreateFixture(b1, circle);
    world.CreateFixture(b2, circle);
    auto jd = RevoluteJointConf{b1, b2, Length2{}};
    jd.enableLimit = true;
    ASSERT_EQ(jd.lowerAngle, 0_deg);
    ASSERT_EQ(jd.upperAngle, 0_deg);

    const auto joint = static_cast<RevoluteJoint*>(world.CreateJoint(jd));
    ASSERT_NE(joint, nullptr);
    ASSERT_EQ(joint->GetLimitState(), Joint::e_inactiveLimit);
    ASSERT_EQ(joint->GetLowerLimit(), jd.lowerAngle);
    ASSERT_EQ(joint->GetUpperLimit(), jd.upperAngle);
    ASSERT_EQ(joint->GetReferenceAngle(), 0_deg);
    ASSERT_EQ(GetJointAngle(*joint), 0_deg);
    
    auto step = StepConf{};
    step.SetTime(1_s);
    step.maxTranslation = Meter * Real(4);
    world.Step(step);

    EXPECT_EQ(GetJointAngle(*joint), 0_deg);
    EXPECT_EQ(joint->GetReferenceAngle(), 0_deg);
    EXPECT_EQ(joint->GetLimitState(), Joint::e_equalLimits);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), -4, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), -4, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
    EXPECT_TRUE(IsEnabled(*joint));
    b1->UnsetAwake();
    b2->UnsetAwake();
    ASSERT_FALSE(b1->IsAwake());
    ASSERT_FALSE(b2->IsAwake());
    SetAwake(*joint);
    EXPECT_TRUE(b1->IsAwake());
    EXPECT_TRUE(b2->IsAwake());
    
    EXPECT_EQ(GetWorldIndex(world, joint), std::size_t(0));
    
    joint->SetLimits(45_deg, 90_deg);
    EXPECT_EQ(joint->GetLowerLimit(), 45_deg);
    EXPECT_EQ(joint->GetUpperLimit(), 90_deg);

    world.Step(step);
    EXPECT_EQ(joint->GetReferenceAngle(), 0_deg);
    EXPECT_EQ(joint->GetLimitState(), Joint::e_atLowerLimit);
    EXPECT_NEAR(static_cast<double>(Real(GetJointAngle(*joint)/1_rad)),
                0.28610128164291382, 0.28610128164291382/100);

    joint->SetLimits(-90_deg, -45_deg);
    EXPECT_EQ(joint->GetLowerLimit(), -90_deg);
    EXPECT_EQ(joint->GetUpperLimit(), -45_deg);
    
    world.Step(step);
    EXPECT_EQ(joint->GetReferenceAngle(), 0_deg);
    EXPECT_EQ(joint->GetLimitState(), Joint::e_atUpperLimit);
    EXPECT_NEAR(static_cast<double>(Real(GetJointAngle(*joint)/1_rad)),
                -0.082102291285991669, 0.082102291285991669/100);
}
#endif

TEST(RevoluteJoint, DynamicJoinedToStaticStaysPut)
{
    auto world = World{};
    
    const auto p1 = Length2{0_m, 4_m}; // Vec2{-1, 0};
    const auto p2 = Length2{0_m, -2_m}; // Vec2{+1, 0};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Static).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));

    const auto shape1 = Shape{PolygonShapeConf{}.SetAsBox(1_m, 1_m)};
    world.CreateFixture(b1, shape1);

    const auto shape2 = Shape{PolygonShapeConf{}.SetAsBox(0.5_m, 0.5_m).UseDensity(1_kgpm2)};
    world.CreateFixture(b2, shape2);

    auto jd = RevoluteJointConf{b1, b2, Length2{}};
    const auto joint = world.CreateJoint(jd);
    
    SetAccelerations(world, Acceleration{
        LinearAcceleration2{0, -10 * MeterPerSquareSecond}, 0 * RadianPerSquareSecond
    });
    for (auto i = 0; i < 1000; ++i)
    {
        Step(world, 0.1_s);
        EXPECT_EQ(GetLocation(world, b1), p1);
        EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}),
                    double(Real{GetX(p2) / Meter}), 0.0001);
        EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}),
                    double(Real{GetY(p2) / Meter}), 0.0001);
        EXPECT_EQ(GetAngle(world, b2), 0_deg);
    }
    
    world.Destroy(joint);
    
    for (auto i = 0; i < 10; ++i)
    {
        Step(world, 0.1_s);
        EXPECT_EQ(GetLocation(world, b1), p1);
        EXPECT_NE(GetLocation(world, b2), p2);
        EXPECT_EQ(GetAngle(world, b2), 0_deg);
    }
}
