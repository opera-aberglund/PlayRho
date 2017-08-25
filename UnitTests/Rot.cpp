/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "gtest/gtest.h"
#include <PlayRho/Common/Math.hpp>

using namespace playrho;

#if 0
TEST(Rot, ByteSizeIs8)
{
    EXPECT_EQ(sizeof(Rot), std::size_t(8));
}

TEST(Rot, sin)
{
    Rot rot0(Angle{0});
    Rot rot90(Radian * Pi / 2);
    Rot rot180(Radian * Pi);
    Rot rot270(Radian * 3 * Pi / 2);
    Rot rot360(Radian * 2 * Pi);
    
    EXPECT_EQ(Real{0}, Round(rot0.sin()));
    EXPECT_EQ(Real{1}, Round(rot90.sin()));
    EXPECT_EQ(Real{0}, Round(rot180.sin()));
    EXPECT_EQ(Real{-1}, Round(rot270.sin()));
    EXPECT_EQ(Real{0}, Round(rot360.sin()));
    EXPECT_EQ(Round(rot0.sin()), Round(rot360.sin()));
    EXPECT_EQ(0.0f, Round(std::asin(rot360.sin())));
}

TEST(Rot, cos)
{
    Rot rot0(Angle{0});
    Rot rot90(Radian * Pi / 2);
    Rot rot180(Radian * 1 * Pi);
    Rot rot360(Radian * 2 * Pi);
    EXPECT_EQ(Round(rot0.cos()), Round(rot360.cos()));
    EXPECT_EQ(Real{1}, Round(rot0.cos()));
    EXPECT_EQ(Real{-1}, Round(rot180.cos()));
    EXPECT_EQ(Real{1}, Round(rot360.cos()));

    EXPECT_EQ(Real{0}, Round(rot90.cos()));
}

TEST(Rot, Add)
{
    Rot rot0(Angle{0});
    Rot rot90(Radian * Pi / 2);
    Rot rot180(Radian * Pi);
    Rot rot270(Radian * 3 * Pi / 2);
    Rot rot360(Radian * 2 * Pi);

    EXPECT_EQ(Round(Real(0)), Round(DegreesToRadians(0)));
    EXPECT_EQ(Round(Pi/2), Round(DegreesToRadians(90)));
    EXPECT_EQ(Round(Pi), Round(DegreesToRadians(180)));
    EXPECT_EQ(Round(3*Pi/2), Round(DegreesToRadians(270)));
    EXPECT_EQ(Round(2*Pi), Round(DegreesToRadians(360)));
    
    EXPECT_EQ(rot0, rot0.Rotate(rot0));    
    EXPECT_EQ(rot90, rot0.Rotate(rot90));
    EXPECT_EQ(rot180, rot90.Rotate(rot90));
    EXPECT_EQ(Round(ToRadians(rot270)), Round(ToRadians(rot180.Rotate(rot90))));
    EXPECT_EQ(Round(ToRadians(Rot(20 * Degree))), Round(ToRadians(Rot(30 * Degree).Rotate(Rot(-10 * Degree)))));
    EXPECT_EQ(Round(ToRadians(Rot(20 * Degree))), Round(ToRadians(Rot(-10 * Degree).Rotate(Rot(30 * Degree)))));
    EXPECT_EQ(Round(ToRadians(Rot(20 * Degree))), Round(ToRadians(Rot(10 * Degree).FlipY().Rotate(Rot(30 * Degree)))));
    EXPECT_EQ(Round(ToRadians(Rot(20 * Degree))), Round(ToRadians(Rot(30 * Degree).Rotate(Rot(10 * Degree).FlipY()))));
    EXPECT_EQ(Round(ToRadians(Rot(105 * Degree))), Round(ToRadians(Rot(45 * Degree).Rotate(Rot(60 * Degree)))));
    EXPECT_EQ(Round(ToRadians(Rot(290 * Degree))), Round(ToRadians(Rot(145 * Degree).Rotate(Rot(145 * Degree)))));
    EXPECT_EQ(Round(ToRadians(Rot(64 * Degree))), Round(ToRadians(Rot(30 * Degree).Rotate(Rot(34 * Degree)))));
}

TEST(Rot, Negate)
{
    EXPECT_EQ(Round(DegreesToRadians(0)), Round(ToRadians(Rot(Angle{0}).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(360 * Degree))), Round(ToRadians(Rot(Angle{0}).FlipY())));
    EXPECT_EQ(-Round(DegreesToRadians(45)), Round(ToRadians(Rot(45 * Degree).FlipY())));
    EXPECT_EQ(-Round(DegreesToRadians(10)), Round(ToRadians(Rot(10 * Degree).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(315 * Degree))), Round(ToRadians(Rot(45 * Degree).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(270 * Degree))), Round(ToRadians(Rot(90 * Degree).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(260 * Degree))), Round(ToRadians(Rot(100 * Degree).FlipY())));
    EXPECT_EQ(-Round(ToRadians(Rot(180 * Degree))), Round(ToRadians(Rot(180 * Degree).FlipY())));
    EXPECT_EQ(Round(ToRadians(Rot(64 * Degree))), Round(ToRadians((Rot(30 * Degree).FlipY()).Rotate(Rot(94 * Degree)))));
    EXPECT_EQ(Round(ToRadians(Rot(-64 * Degree))), Round(ToRadians(Rot(30 * Degree).Rotate(Rot(94 * Degree).FlipY()))));    
}

TEST(Rot, Subtract)
{
    Rot rot0(Angle{0});
    Rot rot90(Radian * Pi / 2);
    Rot rot180(Radian * Pi);
    Rot rot270(Radian * 3 * Pi / 2);
    Rot rot360(Radian * 2 * Pi);

    EXPECT_EQ(Round(ToRadians(rot0)), Round(ToRadians(rot0.Rotate(rot0.FlipY()))));
    EXPECT_EQ(Round(ToRadians(rot90)), Round(ToRadians(rot90.Rotate(rot0.FlipY()))));
    EXPECT_EQ(Round(ToRadians(rot180)), Round(ToRadians(rot180.Rotate(rot0.FlipY()))));
    EXPECT_EQ(Round(ToRadians(rot270)), Round(ToRadians(rot270.Rotate(rot0))));
    
    EXPECT_NE(Round(ToRadians(rot90)), Round(ToRadians(rot0.Rotate(rot90.FlipY()))));
    EXPECT_EQ(Round(ToRadians(rot270)), Round(ToRadians(rot0.Rotate(rot90.FlipY()))));
    EXPECT_EQ(Round(ToRadians(Rot(-90 * Degree))), Round(ToRadians(rot0.Rotate(rot90.FlipY()))));
    EXPECT_EQ(Round(ToRadians(Rot(64 * Degree))), Round(ToRadians(Rot(34 * Degree).Rotate(Rot(-30 * Degree).FlipY()))));
    EXPECT_EQ(Round(ToRadians(Rot(64 * Degree))), Round(ToRadians(Rot(94 * Degree).Rotate(Rot(30 * Degree).FlipY()))));
}
#endif
