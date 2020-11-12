/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "UnitTests.hpp"

#include <PlayRho/Dynamics/Joints/TargetJointConf.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Dynamics/Contacts/ConstraintSolverConf.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>

#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/World.hpp>

#include <stdexcept>

using namespace playrho;
using namespace playrho::d2;

TEST(TargetJointConf, UseTarget)
{
    const auto value = Length2(19_m, -9_m);
    EXPECT_NE(TargetJointConf{}.target, value);
    EXPECT_EQ(TargetJointConf{}.UseTarget(value).target, value);
}

TEST(TargetJointConf, UseMaxForce)
{
    const auto value = Force(19_N);
    EXPECT_NE(TargetJointConf{}.maxForce, value);
    EXPECT_EQ(TargetJointConf{}.UseMaxForce(value).maxForce, value);
}

TEST(TargetJointConf, UseFrequency)
{
    const auto value = 19_Hz;
    EXPECT_NE(TargetJointConf{}.frequency, value);
    EXPECT_EQ(TargetJointConf{}.UseFrequency(value).frequency, value);
}

TEST(TargetJointConf, UseDampingRatio)
{
    const auto value = Real(0.4);
    EXPECT_NE(TargetJointConf{}.dampingRatio, value);
    EXPECT_EQ(TargetJointConf{}.UseDampingRatio(value).dampingRatio, value);
}

TEST(TargetJointConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real)) {
    case 4:
        EXPECT_EQ(sizeof(TargetJointConf), std::size_t(80));
        break;
    case 8:
        EXPECT_EQ(sizeof(TargetJointConf), std::size_t(152));
        break;
    case 16:
        EXPECT_EQ(sizeof(TargetJointConf), std::size_t(304));
        break;
    default:
        FAIL();
        break;
    }
}

#if 0
TEST(TargetJoint, IsOkay)
{
    auto def = TargetJointConf{};

    ASSERT_FALSE(Joint::IsOkay(def));
    def.bodyA = static_cast<BodyID>(1u);
    def.bodyB = static_cast<BodyID>(2u);
    ASSERT_TRUE(Joint::IsOkay(def));

    EXPECT_TRUE(TargetJoint::IsOkay(def));
    def.bodyA = InvalidBodyID;
    EXPECT_TRUE(TargetJoint::IsOkay(def));
    def.target = GetInvalid<decltype(def.target)>();
    EXPECT_FALSE(TargetJoint::IsOkay(def));
}
#endif

TEST(TargetJoint, DefaultInitialized)
{
    const auto def = TargetJointConf{};
    auto joint = Joint{def};

    EXPECT_EQ(GetBodyA(joint), def.bodyA);
    EXPECT_EQ(GetBodyB(joint), def.bodyB);
    EXPECT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    // EXPECT_FALSE(IsValid(joint.GetAnchorB()));
    EXPECT_EQ(GetLinearReaction(joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(joint), AngularMomentum{0});
    EXPECT_FALSE(GetCollideConnected(joint));
    // EXPECT_FALSE(IsValid(GetLocalAnchorB(joint)));
    EXPECT_EQ(GetMaxForce(joint), def.maxForce);
    EXPECT_EQ(GetFrequency(joint), def.frequency);
    EXPECT_EQ(GetDampingRatio(joint), def.dampingRatio);
}

TEST(TargetJoint, GetLocalAnchorB)
{
    auto world = World{};
    const auto bA = world.CreateBody();
    const auto bB = world.CreateBody();
    ASSERT_NE(bA, InvalidBodyID);
    ASSERT_NE(bB, InvalidBodyID);

    auto def = TargetJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.localAnchorB = Length2(-1.4_m, -2_m);
    def.maxForce = 3_N;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);

    const auto joint = Joint{def};
    EXPECT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
}

TEST(TargetJoint, GetAnchorB)
{
    auto world = World{};
    const auto bA = world.CreateBody();
    const auto bB = world.CreateBody();
    ASSERT_NE(bA, InvalidBodyID);
    ASSERT_NE(bB, InvalidBodyID);

    auto def = TargetJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.localAnchorB = Length2(-1.4_m, -2_m);
    def.maxForce = 3_N;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);

    const auto joint = Joint{def};
    ASSERT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
}

TEST(TargetJoint, ShiftOrigin)
{
    auto world = World{};
    const auto bA = world.CreateBody(BodyConf{}.UseLocation(Length2(-1.4_m, -2_m)));
    const auto bB = world.CreateBody();
    ASSERT_NE(bA, InvalidBodyID);
    ASSERT_NE(bB, InvalidBodyID);
    auto def = TargetJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.target = Length2(-1.4_m, -2_m);
    auto joint = Joint{def};
    ASSERT_EQ(GetTarget(joint), def.target);
    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_TRUE(ShiftOrigin(joint, newOrigin));
    EXPECT_EQ(GetTarget(joint), def.target - newOrigin);
}

TEST(TargetJointConf, GetTargetJointDefFreeFunction)
{
    World world;

    const auto bA = world.CreateBody();
    ASSERT_NE(bA, InvalidBodyID);
    const auto bB = world.CreateBody();
    ASSERT_NE(bB, InvalidBodyID);

    auto def = TargetJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.target = Length2(-1.4_m, -2_m);
    def.localAnchorB = Length2(+2.0_m, -1_m);
    def.maxForce = 3_N;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);

    const auto joint = Joint{def};
    const auto got = GetTargetJointConf(joint);

    EXPECT_EQ(def.bodyA, got.bodyA);
    EXPECT_EQ(def.bodyB, got.bodyB);
    EXPECT_EQ(def.target, got.target);
    EXPECT_EQ(def.localAnchorB, got.localAnchorB);
    EXPECT_EQ(def.maxForce, got.maxForce);
    EXPECT_EQ(def.frequency, got.frequency);
    EXPECT_EQ(def.dampingRatio, got.dampingRatio);
}

TEST(TargetJointConf, GetEffectiveMassMatrix)
{
    auto def = TargetJointConf{};
    auto mass = Mass22{};
    EXPECT_NO_THROW(mass = GetEffectiveMassMatrix(def, BodyConstraint{}));
    EXPECT_EQ(mass[0][0], 0_kg);
    EXPECT_EQ(mass[0][1], 0_kg);
    EXPECT_EQ(mass[1][0], 0_kg);
    EXPECT_EQ(mass[1][1], 0_kg);
}

TEST(TargetJointConf, InitVelocityThrows)
{
    std::vector<BodyConstraint> bodies;
    auto step = StepConf{};
    auto conf = ConstraintSolverConf{};
    auto def = TargetJointConf{};
    EXPECT_THROW(InitVelocity(def, bodies, step, conf), std::out_of_range);
}

TEST(TargetJointConf, InitVelocityUpdatesGamma)
{
    auto invMass = InvMass{};
    auto invRotI = InvRotInertia{};
    auto localCenter = Length2{};
    auto position = Position{};
    auto velocity = Velocity{};

    std::vector<BodyConstraint> bodies;
    bodies.push_back(BodyConstraint{invMass, invRotI, localCenter, position, velocity});

    auto step = StepConf{};

    auto conf = ConstraintSolverConf{};

    auto def = TargetJointConf{};
    def.bodyA = BodyID(0u);
    def.bodyB = BodyID(0u);
    def.frequency = 0_Hz;
    def.gamma = Real(5) / 1_kg;
    EXPECT_NO_THROW(InitVelocity(def, bodies, step, conf));
    EXPECT_EQ(def.gamma, Real(0) / 1_kg);

    def.frequency = 1_Hz;
    def.gamma = Real(5) / 1_kg;
    EXPECT_NO_THROW(InitVelocity(def, bodies, step, conf));
    EXPECT_EQ(def.gamma, Real(0) / 1_kg);
}

TEST(TargetJointConf, SolveVelocityThrows)
{
    std::vector<BodyConstraint> bodies;
    auto step = StepConf{};
    auto def = TargetJointConf{};
    EXPECT_THROW(SolveVelocity(def, bodies, step), std::out_of_range);
}

TEST(TargetJointConf, SolvePositionThrows)
{
    std::vector<BodyConstraint> bodies;
    auto conf = ConstraintSolverConf{};
    auto def = TargetJointConf{};
    auto result = false;
    EXPECT_NO_THROW(result = SolvePosition(def, bodies, conf));
    EXPECT_EQ(result, true);
}

TEST(TargetJointConf, EqualsOperator)
{
    EXPECT_TRUE(TargetJointConf() == TargetJointConf());
    {
        auto conf = TargetJointConf{};
        conf.target = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(TargetJointConf() == conf);
    }
    {
        auto conf = TargetJointConf{};
        conf.localAnchorB = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(TargetJointConf() == conf);
    }
    {
        auto conf = TargetJointConf{};
        conf.maxForce = 12_N;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(TargetJointConf() == conf);
    }
    // TODO: test remaining fields.
}

TEST(TargetJointConf, NotEqualsOperator)
{
    EXPECT_FALSE(TargetJointConf() != TargetJointConf());
    {
        auto conf = TargetJointConf{};
        conf.frequency = 13_Hz;
        EXPECT_FALSE(conf != conf);
        EXPECT_TRUE(TargetJointConf() != conf);
    }
    // TODO: test remaining fields.
}

TEST(TargetJointConf, GetName)
{
    EXPECT_STREQ(GetName(GetTypeID<TargetJointConf>()), "d2::TargetJointConf");
}
