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

#include <playrho/d2/PolygonShapeConf.hpp>
#include <playrho/d2/Shape.hpp>

#include "gtest/gtest.h"

using namespace playrho;
using namespace playrho::d2;

TEST(PolygonShapeConf, DefaultConstruction)
{
    EXPECT_EQ(PolygonShapeConf::GetDefaultVertexRadius(), PolygonShapeConf::DefaultVertexRadius);
    const auto shape = PolygonShapeConf{};
    EXPECT_EQ(shape.GetVertexCount(), 0);
    EXPECT_EQ(GetChildCount(shape), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());
    EXPECT_FALSE(IsValid(ComputeCentroid(shape.GetVertices())));
}

TEST(PolygonShapeConf, GetInvalidChildThrows)
{
    const auto foo = PolygonShapeConf{};
    
    ASSERT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_NO_THROW(GetChild(foo, 0));
    EXPECT_THROW(GetChild(foo, 1), InvalidArgument);
}

TEST(PolygonShapeConf, TypeInfo)
{
    const auto foo = PolygonShapeConf{};
    const auto shape = Shape(foo);
    EXPECT_EQ(GetType(shape), GetTypeID<PolygonShapeConf>());
    auto copy = PolygonShapeConf{};
    EXPECT_NO_THROW(copy = TypeCast<PolygonShapeConf>(shape));
    EXPECT_THROW(TypeCast<int>(shape), std::bad_cast);
}

TEST(PolygonShapeConf, FindLowestRightMostVertex)
{
    Length2 vertices[4];
    
    vertices[0] = Length2{0_m, +1_m};
    vertices[1] = Vec2{-1, -2} * Meter;
    vertices[2] = Vec2{+3, -4} * Meter;
    vertices[3] = Vec2{+2, +2} * Meter;

    const auto index = FindLowestRightMostVertex(vertices);
    
    EXPECT_EQ(index, std::size_t(2));
}

TEST(PolygonShapeConf, BoxConstruction)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    const auto shape = PolygonShapeConf{hx, hy};

    EXPECT_EQ(ComputeCentroid(shape.GetVertices()), (Length2{}));
    EXPECT_EQ(GetChildCount(shape), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());

    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...

    EXPECT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left

    EXPECT_DOUBLE_EQ(shape.GetNormal(0).GetX(), GetX(Vec2(+1, 0)));
    EXPECT_DOUBLE_EQ(shape.GetNormal(0).GetY(), GetY(Vec2(+1, 0)));

    EXPECT_DOUBLE_EQ(shape.GetNormal(1).GetX(), GetX(Vec2(0, +1)));
    EXPECT_DOUBLE_EQ(shape.GetNormal(1).GetY(), GetY(Vec2(0, +1)));

    EXPECT_DOUBLE_EQ(shape.GetNormal(2).GetX(), GetX(Vec2(-1, 0)));
    EXPECT_DOUBLE_EQ(shape.GetNormal(2).GetY(), GetY(Vec2(-1, 0)));

    EXPECT_DOUBLE_EQ(shape.GetNormal(3).GetX(), GetX(Vec2(0, -1)));
    EXPECT_DOUBLE_EQ(shape.GetNormal(3).GetY(), GetY(Vec2(0, -1)));

    EXPECT_TRUE(Validate(shape.GetVertices()));
}

TEST(PolygonShapeConf, Copy)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    
    auto shape = PolygonShapeConf{hx, hy};
    ASSERT_EQ(ComputeCentroid(shape.GetVertices()), (Length2{}));
    ASSERT_EQ(GetChildCount(shape), ChildCounter(1));
    ASSERT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    ASSERT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    ASSERT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    ASSERT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    ASSERT_DOUBLE_EQ(shape.GetNormal(0).GetX(), Real(+1));
    ASSERT_DOUBLE_EQ(shape.GetNormal(0).GetY(), Real(0));
    ASSERT_DOUBLE_EQ(shape.GetNormal(1).GetX(), Real(0));
    ASSERT_DOUBLE_EQ(shape.GetNormal(1).GetY(), Real(+1));
    ASSERT_DOUBLE_EQ(shape.GetNormal(2).GetX(), Real(-1));
    ASSERT_DOUBLE_EQ(shape.GetNormal(2).GetY(), Real(0));
    ASSERT_DOUBLE_EQ(shape.GetNormal(3).GetX(), Real(0));
    ASSERT_DOUBLE_EQ(shape.GetNormal(3).GetY(), Real(-1));

    const auto copy = shape;
    
    EXPECT_EQ(GetTypeID(copy), GetTypeID(shape));
    EXPECT_EQ(ComputeCentroid(copy.GetVertices()), (Length2{}));
    EXPECT_EQ(GetChildCount(copy), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(copy), PolygonShapeConf::GetDefaultVertexRadius());
    
    ASSERT_EQ(copy.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(copy.GetVertex(0), Length2(hx, -hy)); // bottom right
    EXPECT_EQ(copy.GetVertex(1), Length2(hx, hy)); // top right
    EXPECT_EQ(copy.GetVertex(2), Length2(-hx, hy)); // top left
    EXPECT_EQ(copy.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    EXPECT_DOUBLE_EQ(copy.GetNormal(0).GetX(), Real(+1));
    EXPECT_DOUBLE_EQ(copy.GetNormal(0).GetY(), Real(0));
    EXPECT_DOUBLE_EQ(copy.GetNormal(1).GetX(), Real(0));
    EXPECT_DOUBLE_EQ(copy.GetNormal(1).GetY(), Real(+1));
    EXPECT_DOUBLE_EQ(copy.GetNormal(2).GetX(), Real(-1));
    EXPECT_DOUBLE_EQ(copy.GetNormal(2).GetY(), Real(0));
    EXPECT_DOUBLE_EQ(copy.GetNormal(3).GetX(), Real(0));
    EXPECT_DOUBLE_EQ(copy.GetNormal(3).GetY(), Real(-1));
}

TEST(PolygonShapeConf, Transform)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    
    auto shape = PolygonShapeConf{hx, hy};
    ASSERT_EQ(ComputeCentroid(shape.GetVertices()), (Length2{}));
    ASSERT_EQ(GetChildCount(shape), ChildCounter(1));
    ASSERT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    ASSERT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    ASSERT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    ASSERT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    ASSERT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    ASSERT_DOUBLE_EQ(shape.GetNormal(0).GetX(), Real(+1));
    ASSERT_DOUBLE_EQ(shape.GetNormal(0).GetY(), Real(0));
    ASSERT_DOUBLE_EQ(shape.GetNormal(1).GetX(), Real(0));
    ASSERT_DOUBLE_EQ(shape.GetNormal(1).GetY(), Real(+1));
    ASSERT_DOUBLE_EQ(shape.GetNormal(2).GetX(), Real(-1));
    ASSERT_DOUBLE_EQ(shape.GetNormal(2).GetY(), Real(0));
    ASSERT_DOUBLE_EQ(shape.GetNormal(3).GetX(), Real(0));
    ASSERT_DOUBLE_EQ(shape.GetNormal(3).GetY(), Real(-1));

    const auto new_ctr = Length2{-3_m, 67_m};
    shape = PolygonShapeConf{
        PolygonShapeConf{}.SetAsBox(hx, hy).Transform(Transformation{new_ctr, UnitVec::GetRight()})
    };

    const auto centroid = ComputeCentroid(shape.GetVertices());
    EXPECT_NEAR(static_cast<double>(Real{GetX(centroid)/Meter}),
                static_cast<double>(Real{GetX(new_ctr)/Meter}), 0.001);
    EXPECT_NEAR(static_cast<double>(Real{GetY(centroid)/Meter}),
                static_cast<double>(Real{GetY(new_ctr)/Meter}), 0.001);
    EXPECT_EQ(GetChildCount(shape), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());

    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));

    EXPECT_EQ(shape.GetVertex(0), Length2(hx, -hy) + new_ctr); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx, hy) + new_ctr); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx, hy) + new_ctr); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx, -hy) + new_ctr); // bottom left

    EXPECT_EQ(shape.GetNormal(0) * Real{1}, Vec2(+1, 0));
    EXPECT_EQ(shape.GetNormal(1) * Real{1}, Vec2(0, +1));
    EXPECT_EQ(shape.GetNormal(2) * Real{1}, Vec2(-1, 0));
    EXPECT_EQ(shape.GetNormal(3) * Real{1}, Vec2(0, -1));
}

TEST(PolygonShapeConf, SetAsBox)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    const auto shape = PolygonShapeConf{hx, hy};
    EXPECT_EQ(ComputeCentroid(shape.GetVertices()), (Length2{}));
    EXPECT_EQ(GetChildCount(shape), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    EXPECT_DOUBLE_EQ(shape.GetNormal(0).GetX(), Real(+1));
    EXPECT_DOUBLE_EQ(shape.GetNormal(0).GetY(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(1).GetX(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(1).GetY(), Real(+1));
    EXPECT_DOUBLE_EQ(shape.GetNormal(2).GetX(), Real(-1));
    EXPECT_DOUBLE_EQ(shape.GetNormal(2).GetY(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(3).GetX(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(3).GetY(), Real(-1));
}

TEST(PolygonShapeConf, SetAsZeroCenteredRotatedBox)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    const auto shape = PolygonShapeConf{PolygonShapeConf{}.SetAsBox(hx, hy, Length2{}, 0_deg)};
    EXPECT_EQ(ComputeCentroid(shape.GetVertices()), (Length2{}));
    EXPECT_EQ(GetChildCount(shape), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(shape.GetVertex(0), Length2(hx, -hy)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx, hy)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx, hy)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx, -hy)); // bottom left
    
    EXPECT_DOUBLE_EQ(shape.GetNormal(0).GetX(), Real(+1));
    EXPECT_DOUBLE_EQ(shape.GetNormal(0).GetY(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(1).GetX(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(1).GetY(), Real(+1));
    EXPECT_DOUBLE_EQ(shape.GetNormal(2).GetX(), Real(-1));
    EXPECT_DOUBLE_EQ(shape.GetNormal(2).GetY(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(3).GetX(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(3).GetY(), Real(-1));
}

TEST(PolygonShapeConf, SetAsCenteredBox)
{
    const auto hx = 2.3_m;
    const auto hy = 54.1_m;
    const auto x_off = 10.2_m;
    const auto y_off = -5_m;
    const auto shape = PolygonShapeConf{PolygonShapeConf{}.SetAsBox(hx, hy, Length2(x_off, y_off), 0_deg)};
    const auto centroid = ComputeCentroid(shape.GetVertices());
    EXPECT_NEAR(static_cast<double>(Real{GetX(centroid)/Meter}),
                static_cast<double>(Real{x_off/Meter}), 0.001);
    EXPECT_NEAR(static_cast<double>(Real{GetY(centroid)/Meter}),
                static_cast<double>(Real{y_off/Meter}), 0.001);
    EXPECT_EQ(GetChildCount(shape), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
    
    EXPECT_EQ(shape.GetVertex(0), Length2(hx + x_off, -hy + y_off)); // bottom right
    EXPECT_EQ(shape.GetVertex(1), Length2(hx + x_off, hy + y_off)); // top right
    EXPECT_EQ(shape.GetVertex(2), Length2(-hx + x_off, hy + y_off)); // top left
    EXPECT_EQ(shape.GetVertex(3), Length2(-hx + x_off, -hy + y_off)); // bottom left
    
    EXPECT_DOUBLE_EQ(shape.GetNormal(0).GetX(), Real(+1));
    EXPECT_DOUBLE_EQ(shape.GetNormal(0).GetY(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(1).GetX(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(1).GetY(), Real(+1));
    EXPECT_DOUBLE_EQ(shape.GetNormal(2).GetX(), Real(-1));
    EXPECT_DOUBLE_EQ(shape.GetNormal(2).GetY(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(3).GetX(), Real(0));
    EXPECT_DOUBLE_EQ(shape.GetNormal(3).GetY(), Real(-1));
}

TEST(PolygonShapeConf, SetAsBoxAngledDegrees90)
{
    const auto hx = Real(2.3);
    const auto hy = Real(54.1);
    const auto angle = 90.01_deg;
    const auto shape = PolygonShapeConf{PolygonShapeConf{}.SetAsBox(hx * Meter, hy * Meter, Length2{}, angle)};

    const auto centroid = ComputeCentroid(shape.GetVertices());
    EXPECT_NEAR(static_cast<double>(Real{GetX(centroid)/Meter}), 0.0, 0.01);
    EXPECT_NEAR(static_cast<double>(Real{GetY(centroid)/Meter}), 0.0, 0.01);
    EXPECT_EQ(GetChildCount(shape), ChildCounter(1));
    EXPECT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());
    
    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(4));
    
    // vertices go counter-clockwise (and normals follow their edges)...
    
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(3)) / Meter}), double( hy), 0.02); // right
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(3)) / Meter}), double(-hx), 0.02); // bottom
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(0)) / Meter}), double( hy), 0.02); // right
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(0)) / Meter}), double( hx), 0.02); // top
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(1)) / Meter}), double(-hy), 0.02); // left
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(1)) / Meter}), double( hx), 0.02); // top
    EXPECT_NEAR(double(Real{GetX(shape.GetVertex(2)) / Meter}), double(-hy), 0.02); // left
    EXPECT_NEAR(double(Real{GetY(shape.GetVertex(2)) / Meter}), double(-hx), 0.02); // bottom
    
    EXPECT_NEAR(double(shape.GetNormal(3).GetX()), +1.0, 0.01);
    EXPECT_NEAR(double(shape.GetNormal(3).GetY()),  0.0, 0.01);
    EXPECT_NEAR(double(shape.GetNormal(0).GetX()),  0.0, 0.01);
    EXPECT_NEAR(double(shape.GetNormal(0).GetY()), +1.0, 0.01);
    EXPECT_NEAR(double(shape.GetNormal(1).GetX()), -1.0, 0.01);
    EXPECT_NEAR(double(shape.GetNormal(1).GetY()),  0.0, 0.01);
    EXPECT_NEAR(double(shape.GetNormal(2).GetX()),  0.0, 0.01);
    EXPECT_NEAR(double(shape.GetNormal(2).GetY()), -1.0, 0.01);
}

TEST(PolygonShapeConf, SetPoints)
{
    const auto points = Vector<const Length2, 5>{
        Vec2{-1, +2} * Meter,
        Vec2{+3, +3} * Meter,
        Vec2{+2, -1} * Meter,
        Vec2{-1, -2} * Meter,
        Vec2{-4, -1} * Meter
    };
    const auto shape = PolygonShapeConf{}.Set(points);

    ASSERT_EQ(shape.GetVertexCount(), VertexCounter(5));

    // vertices go counter-clockwise from lowest right-most...

    EXPECT_EQ(shape.GetVertex(0), points[1]);
    EXPECT_EQ(shape.GetVertex(1), points[0]);
    EXPECT_EQ(shape.GetVertex(2), points[4]);
    EXPECT_EQ(shape.GetVertex(3), points[3]);
    EXPECT_EQ(shape.GetVertex(4), points[2]);
    
    EXPECT_TRUE(Validate(shape.GetVertices()));
}

TEST(PolygonShapeConf, UseVertices)
{
    const auto p0 = Length2{1_m, 2_m};
    const auto p1 = Length2{3_m, 4_m};

    auto conf = PolygonShapeConf{};
    ASSERT_EQ(conf.GetVertexCount(), 0);
    conf.UseVertices(std::vector<Length2>{});
    EXPECT_EQ(conf.GetVertexCount(), 0);
    EXPECT_FALSE(IsValid(ComputeCentroid(conf.GetVertices())));
    conf.UseVertices(std::vector<Length2>{p0});
    EXPECT_EQ(conf.GetVertexCount(), 1);
    EXPECT_EQ(conf.GetVertex(0), p0);
    EXPECT_TRUE(IsValid(ComputeCentroid(conf.GetVertices())));
    conf.UseVertices(std::vector<Length2>{p0, p1});
    EXPECT_EQ(conf.GetVertexCount(), 2);
    EXPECT_EQ(conf.GetVertex(0), p1);
    EXPECT_EQ(conf.GetVertex(1), p0);
    EXPECT_TRUE(IsValid(ComputeCentroid(conf.GetVertices())));
}

TEST(PolygonShapeConf, CanSetTwoPoints)
{
    const auto points = Vector<const Length2, 2>{
        Vec2{-1, +0} * Meter,
        Vec2{+1, +0} * Meter
    };
    const auto vertexRadius = 2_m;
    const auto shape = PolygonShapeConf{PolygonShapeConf{}.UseVertexRadius(vertexRadius).Set(points)};
    EXPECT_EQ(shape.GetVertexCount(), static_cast<VertexCounter>(points.size()));
    EXPECT_EQ(shape.GetVertex(0), points[1]);
    EXPECT_EQ(shape.GetVertex(1), points[0]);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(shape.GetNormal(0)))),
                +0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(shape.GetNormal(0)))),
                +1.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetX(GetVec2(shape.GetNormal(1)))),
                +0.0, 1.0/100000.0);
    EXPECT_NEAR(static_cast<double>(GetY(GetVec2(shape.GetNormal(1)))),
                -1.0, 1.0/100000.0);
    EXPECT_EQ(ComputeCentroid(shape.GetVertices()), Average(points));
    EXPECT_EQ(GetVertexRadius(shape), vertexRadius);

    EXPECT_TRUE(Validate(shape.GetVertices()));
}

TEST(PolygonShapeConf, CanSetOnePoint)
{
    const auto points = Vector<const Length2, 1>{Length2{}};
    const auto vertexRadius = 2_m;
    const auto shape = PolygonShapeConf{PolygonShapeConf{}.UseVertexRadius(vertexRadius).Set(points)};
    EXPECT_EQ(shape.GetVertexCount(), static_cast<VertexCounter>(points.size()));
    EXPECT_EQ(shape.GetVertex(0), points[0]);
    EXPECT_FALSE(IsValid(shape.GetNormal(0)));
    EXPECT_EQ(ComputeCentroid(shape.GetVertices()), points[0]);
    EXPECT_EQ(GetVertexRadius(shape), vertexRadius);
}

TEST(PolygonShapeConf, TransformFF)
{
    {
        auto foo = PolygonShapeConf{};
        auto copy = foo;
        Transform(foo, Mat22{});
        EXPECT_EQ(foo, copy);
    }
    {
        auto foo = PolygonShapeConf{};
        auto copy = foo;
        Transform(foo, GetIdentity<Mat22>());
        EXPECT_EQ(foo, copy);
    }
    {
        const auto v1 = Length2{1_m, 2_m};
        const auto v2 = Length2{3_m, 4_m};
        const auto vertices = std::vector<Length2>{v1, v2};
        auto foo = PolygonShapeConf{vertices};
        auto copy = foo;
        Transform(foo, GetIdentity<Mat22>());
        EXPECT_EQ(foo, copy);
    }
    {
        const auto v1 = Length2{1_m, 2_m};
        const auto v2 = Length2{3_m, 4_m};
        const auto vertices = std::vector<Length2>{v1, v2};
        auto foo = PolygonShapeConf{vertices};
        ASSERT_EQ(foo.GetVertexCount(), VertexCounter(2));
        ASSERT_EQ(foo.GetVertex(0), v2);
        ASSERT_EQ(foo.GetVertex(1), v1);
        auto copy = foo;
        Transform(foo, GetIdentity<Mat22>() * 2);
        EXPECT_EQ(foo.GetVertex(0), v2 * 2);
        EXPECT_EQ(foo.GetVertex(1), v1 * 2);
    }
}

TEST(PolygonShapeConf, Equality)
{
    EXPECT_TRUE(PolygonShapeConf() == PolygonShapeConf());

    EXPECT_FALSE(PolygonShapeConf().SetAsBox(1_m, 2_m) == PolygonShapeConf());
    EXPECT_TRUE(PolygonShapeConf().SetAsBox(1_m, 2_m) == PolygonShapeConf().SetAsBox(1_m, 2_m));

    EXPECT_FALSE(PolygonShapeConf().UseVertexRadius(10_m) == PolygonShapeConf());
    EXPECT_TRUE(PolygonShapeConf().UseVertexRadius(10_m) == PolygonShapeConf().UseVertexRadius(10_m));
    
    EXPECT_FALSE(PolygonShapeConf().UseDensity(10_kgpm2) == PolygonShapeConf());
    EXPECT_TRUE(PolygonShapeConf().UseDensity(10_kgpm2) == PolygonShapeConf().UseDensity(10_kgpm2));
    
    EXPECT_FALSE(PolygonShapeConf().UseFriction(Real(10)) == PolygonShapeConf());
    EXPECT_TRUE(PolygonShapeConf().UseFriction(Real(10)) == PolygonShapeConf().UseFriction(Real(10)));
    
    EXPECT_FALSE(PolygonShapeConf().UseRestitution(Real(10)) == PolygonShapeConf());
    EXPECT_TRUE(PolygonShapeConf().UseRestitution(Real(10)) == PolygonShapeConf().UseRestitution(Real(10)));
}

TEST(PolygonShapeConf, Inequality)
{
    EXPECT_FALSE(PolygonShapeConf() != PolygonShapeConf());
    
    EXPECT_TRUE(PolygonShapeConf().SetAsBox(1_m, 2_m) != PolygonShapeConf());
    EXPECT_FALSE(PolygonShapeConf().SetAsBox(1_m, 2_m) != PolygonShapeConf().SetAsBox(1_m, 2_m));

    EXPECT_TRUE(PolygonShapeConf().UseVertexRadius(10_m) != PolygonShapeConf());
    EXPECT_FALSE(PolygonShapeConf().UseVertexRadius(10_m) != PolygonShapeConf().UseVertexRadius(10_m));
    
    EXPECT_TRUE(PolygonShapeConf().UseDensity(10_kgpm2) != PolygonShapeConf());
    EXPECT_FALSE(PolygonShapeConf().UseDensity(10_kgpm2) != PolygonShapeConf().UseDensity(10_kgpm2));
    
    EXPECT_TRUE(PolygonShapeConf().UseFriction(Real(10)) != PolygonShapeConf());
    EXPECT_FALSE(PolygonShapeConf().UseFriction(Real(10)) != PolygonShapeConf().UseFriction(Real(10)));
    
    EXPECT_TRUE(PolygonShapeConf().UseRestitution(Real(10)) != PolygonShapeConf());
    EXPECT_FALSE(PolygonShapeConf().UseRestitution(Real(10)) != PolygonShapeConf().UseRestitution(Real(10)));
}

TEST(PolygonShapeConf, ValidateFF)
{
    auto vertices = std::vector<Length2>{};
    EXPECT_TRUE(Validate(vertices));
    vertices.push_back(Length2{0_m, 0_m});
    EXPECT_TRUE(Validate(vertices));
    vertices.push_back(Length2{1_m, 1_m});
    EXPECT_TRUE(Validate(vertices));
    vertices.push_back(Length2{-1_m, 1_m});
    EXPECT_TRUE(Validate(vertices));
    vertices.push_back(Length2{+2_m, 1_m});
    EXPECT_FALSE(Validate(vertices));
}

TEST(PolygonShapeConf, SetVertexRadius)
{
    auto shape = PolygonShapeConf{};
    ASSERT_EQ(shape.GetVertexCount(), 0);
    ASSERT_EQ(GetChildCount(shape), ChildCounter(1));
    ASSERT_EQ(GetVertexRadius(shape), PolygonShapeConf::GetDefaultVertexRadius());
    EXPECT_FALSE(IsValid(ComputeCentroid(shape.GetVertices())));
    const auto amount = 2_m;
    EXPECT_NO_THROW(SetVertexRadius(shape, 0u, amount));
    EXPECT_EQ(GetVertexRadius(shape), amount);
}

TEST(PolygonShapeConf, Translate)
{
    const auto v0 = Length2{-1.0_m, 0_m};
    const auto v1 = Length2{+1.0_m, 0_m};
    auto vertices = VertexSet{};
    vertices.add(v0);
    vertices.add(v1);
    auto shape = PolygonShapeConf{};
    shape.Set(vertices);
    ASSERT_EQ(shape.GetVertexCount(), 2u);
    ASSERT_EQ(shape.GetVertex(0u), v1);
    ASSERT_EQ(shape.GetVertex(1u), v0);
    const auto amount = Length2{2_m, 3_m};
    EXPECT_NO_THROW(Translate(shape, amount));
    ASSERT_EQ(shape.GetVertexCount(), 2u);
    EXPECT_EQ(shape.GetVertex(0u), v1 + amount);
    EXPECT_EQ(shape.GetVertex(1u), v0 + amount);
}

TEST(PolygonShapeConf, Scale)
{
    const auto v0 = Length2{-1.0_m, 0_m};
    const auto v1 = Length2{+1.0_m, 0_m};
    auto vertices = VertexSet{};
    vertices.add(v0);
    vertices.add(v1);
    auto shape = PolygonShapeConf{};
    shape.Set(vertices);
    ASSERT_EQ(shape.GetVertexCount(), 2u);
    ASSERT_EQ(shape.GetVertex(0u), v1);
    ASSERT_EQ(shape.GetVertex(1u), v0);
    const auto amount = Vec2{Real(2), Real(3)};
    EXPECT_NO_THROW(Scale(shape, amount));
    ASSERT_EQ(shape.GetVertexCount(), 2u);
    EXPECT_EQ(shape.GetVertex(1u), Length2(GetX(v0) * GetX(amount), GetY(v0) * GetY(amount)));
    EXPECT_EQ(shape.GetVertex(0u), Length2(GetX(v1) * GetX(amount), GetY(v1) * GetY(amount)));
}

TEST(PolygonShapeConf, Rotate)
{
    const auto v0 = Length2{+1.0_m, 0_m};
    const auto v1 = Length2{-1.0_m, 0_m};
    auto vertices = VertexSet{};
    vertices.add(v0);
    vertices.add(v1);
    auto shape = PolygonShapeConf{};
    shape.Set(vertices);
    ASSERT_EQ(shape.GetVertexCount(), 2u);
    ASSERT_EQ(shape.GetVertex(0u), v0);
    ASSERT_EQ(shape.GetVertex(1u), v1);
    const auto amount = UnitVec::GetUp();
    EXPECT_NO_THROW(Rotate(shape, amount));
    ASSERT_EQ(shape.GetVertexCount(), 2u);
    EXPECT_EQ(shape.GetVertex(0u), Rotate(v1, amount));
    EXPECT_EQ(shape.GetVertex(1u), Rotate(v0, amount));
}
