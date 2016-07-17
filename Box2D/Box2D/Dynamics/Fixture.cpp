/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/World.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/BroadPhase.h>
#include <Box2D/Common/BlockAllocator.h>

using namespace box2d;

void Fixture::Create(BlockAllocator* allocator, const FixtureDef& def)
{
	assert(def.density >= 0);
	m_userData = def.userData;
	m_friction = def.friction;
	m_restitution = def.restitution;
	m_filter = def.filter;
	m_isSensor = def.isSensor;
	m_density = Max(def.density, float_t{0});
	m_shape = def.shape->Clone(allocator);

	// Reserve proxy space
	const auto childCount = m_shape->GetChildCount();
	const auto proxies = allocator->AllocateArray<FixtureProxy>(childCount);
	for (auto i = decltype(childCount){0}; i < childCount; ++i)
	{
		proxies[i].fixture = nullptr;
		proxies[i].proxyId = BroadPhase::e_nullProxy;
	}
	m_proxies = proxies;
}

template <>
inline void box2d::Delete(Shape* shape, BlockAllocator& allocator)
{
	switch (shape->GetType())
	{
		case Shape::e_circle:
			Delete(static_cast<CircleShape*>(shape), allocator);
			break;
		case Shape::e_edge:
			Delete(static_cast<EdgeShape*>(shape), allocator);
			break;
		case Shape::e_polygon:
			Delete(static_cast<PolygonShape*>(shape), allocator);
			break;
		case Shape::e_chain:
			Delete(static_cast<ChainShape*>(shape), allocator);
			break;
		default:
			assert(false);
			break;
	}
}

void Fixture::Destroy(BlockAllocator* allocator)
{
	// The proxies must be destroyed before calling this.
	assert(m_proxyCount == 0);

	// Free the proxy array.
	const auto childCount = m_shape->GetChildCount();
	allocator->Free(m_proxies, childCount * sizeof(FixtureProxy));
	m_proxies = nullptr;

	// Free the child shape.
	Delete(m_shape, *allocator);
	m_shape = nullptr;
}

void Fixture::CreateProxies(BroadPhase& broadPhase, const Transformation& xf)
{
	assert(m_proxyCount == 0);

	m_proxyCount = m_shape->GetChildCount();

	// Create proxies in the broad-phase.
	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		const auto aabb = m_shape->ComputeAABB(xf, i);
		m_proxies[i] = FixtureProxy{aabb, broadPhase.CreateProxy(aabb, m_proxies + i), this, i};
	}
}

void Fixture::DestroyProxies(BroadPhase& broadPhase)
{
	// Destroy proxies in the broad-phase.
	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		auto& proxy = m_proxies[i];
		broadPhase.DestroyProxy(proxy.proxyId);
		proxy.proxyId = BroadPhase::e_nullProxy;
	}

	m_proxyCount = 0;
}

void Fixture::Synchronize(BroadPhase& broadPhase, const Transformation& transform1, const Transformation& transform2)
{
	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		auto& proxy = m_proxies[i];

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		const auto aabb1 = m_shape->ComputeAABB(transform1, proxy.childIndex);
		const auto aabb2 = m_shape->ComputeAABB(transform2, proxy.childIndex);
		proxy.aabb = aabb1 + aabb2;

		broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, transform2.p - transform1.p);
	}
}

void Fixture::SetFilterData(const Filter& filter)
{
	m_filter = filter;

	Refilter();
}

void Fixture::Refilter()
{
	if (m_body == nullptr)
	{
		return;
	}

	// Flag associated contacts for filtering.
	for (auto&& edge: m_body->GetContactEdges())
	{
		auto contact = edge.contact;
		const auto fixtureA = contact->GetFixtureA();
		const auto fixtureB = contact->GetFixtureB();
		if ((fixtureA == this) || (fixtureB == this))
		{
			contact->FlagForFiltering();
		}
	}

	auto world = m_body->GetWorld();

	if (world == nullptr)
	{
		return;
	}

	// Touch each proxy so that new pairs may be created
	auto broadPhase = &world->m_contactMgr.m_broadPhase;
	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		broadPhase->TouchProxy(m_proxies[i].proxyId);
	}
}

void Fixture::SetSensor(bool sensor)
{
	if (sensor != m_isSensor)
	{
		m_body->SetAwake();
		m_isSensor = sensor;
	}
}

void Fixture::Dump(island_count_t bodyIndex)
{
	log("    FixtureDef fd;\n");
	log("    fd.friction = %.15lef;\n", m_friction);
	log("    fd.restitution = %.15lef;\n", m_restitution);
	log("    fd.density = %.15lef;\n", m_density);
	log("    fd.isSensor = bool(%d);\n", m_isSensor);
	log("    fd.filter.categoryBits = uint16(%d);\n", m_filter.categoryBits);
	log("    fd.filter.maskBits = uint16(%d);\n", m_filter.maskBits);
	log("    fd.filter.groupIndex = int16(%d);\n", m_filter.groupIndex);

	switch (m_shape->GetType())
	{
	case Shape::e_circle:
		{
			auto s = static_cast<CircleShape*>(m_shape);
			log("    CircleShape shape;\n");
			log("    shape.m_radius = %.15lef;\n", s->GetRadius());
			log("    shape.m_p = Vec2(%.15lef, %.15lef);\n", s->GetPosition().x, s->GetPosition().y);
		}
		break;

	case Shape::e_edge:
		{
			auto s = static_cast<EdgeShape*>(m_shape);
			log("    EdgeShape shape;\n");
			log("    shape.m_radius = %.15lef;\n", s->GetRadius());
			log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s->GetVertex0().x, s->GetVertex0().y);
			log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s->GetVertex1().x, s->GetVertex1().y);
			log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s->GetVertex2().x, s->GetVertex2().y);
			log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s->GetVertex3().x, s->GetVertex3().y);
			log("    shape.m_hasVertex0 = bool(%d);\n", s->HasVertex0());
			log("    shape.m_hasVertex3 = bool(%d);\n", s->HasVertex3());
		}
		break;

	case Shape::e_polygon:
		{
			auto s = static_cast<PolygonShape*>(m_shape);
			log("    PolygonShape shape;\n");
			log("    Vec2 vs[%d];\n", MaxPolygonVertices);
			for (auto i = decltype(s->GetVertexCount()){0}; i < s->GetVertexCount(); ++i)
			{
				log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->GetVertex(i).x, s->GetVertex(i).y);
			}
			log("    shape.Set(vs, %d);\n", s->GetVertexCount());
		}
		break;

	case Shape::e_chain:
		{
			auto s = static_cast<ChainShape*>(m_shape);
			log("    ChainShape shape;\n");
			log("    Vec2 vs[%d];\n", s->GetVertexCount());
			for (auto i = decltype(s->GetVertexCount()){0}; i < s->GetVertexCount(); ++i)
			{
				log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->GetVertex(i).x, s->GetVertex(i).y);
			}
			log("    shape.CreateChain(vs, %d);\n", s->GetVertexCount());
			log("    shape.m_prevVertex.Set(%.15lef, %.15lef);\n", s->GetPrevVertex().x, s->GetPrevVertex().y);
			log("    shape.m_nextVertex.Set(%.15lef, %.15lef);\n", s->GetNextVertex().x, s->GetNextVertex().y);
			log("    shape.m_hasPrevVertex = bool(%d);\n", s->HasPrevVertex());
			log("    shape.m_hasNextVertex = bool(%d);\n", s->HasNextVertex());
		}
		break;

	default:
		return;
	}

	log("\n");
	log("    fd.shape = &shape;\n");
	log("\n");
	log("    bodies[%d]->CreateFixture(fd);\n", bodyIndex);
}
