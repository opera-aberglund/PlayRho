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

#ifndef B2_CONTACT_SOLVER_H
#define B2_CONTACT_SOLVER_H

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/Collision.h>
#include <Box2D/Dynamics/TimeStep.h>

namespace box2d {

class Contact;
class Body;
class StackAllocator;
struct ContactPositionConstraint;
struct ContactPositionConstraintBodyData;
class Fixture;

/// Velocity constraint point.
/// @note This structure is at least 36-bytes large.
struct VelocityConstraintPoint
{
	Vec2 rA; ///< Position of body A relative to world manifold point (8-bytes).
	Vec2 rB; ///< Position of body B relative to world manifold point (8-bytes).
	float_t normalImpulse; ///< Normal impulse (4-bytes).
	float_t tangentImpulse; ///< Tangent impulse (4-bytes).
	float_t normalMass; ///< Normal mass (4-bytes).
	float_t tangentMass; ///< Tangent mass (4-bytes).
	float_t velocityBias; ///< Velocity bias (4-bytes).
};

/// Contact velocity constraint.
/// @note A valid contact velocity constraint must have a point count of either 1 or 2.
/// @note This data structure is at least 125-bytes large.
class ContactVelocityConstraint
{
public:
	using size_type = std::remove_const<decltype(MaxManifoldPoints)>::type;
	using index_type = size_t;

	/// Contact velocity constraint body data.
	class BodyData
	{
	public:
		BodyData() noexcept = default;
		BodyData(const BodyData& copy) noexcept = default;
		
		constexpr BodyData(index_type i, float_t iM, float_t iI) noexcept: index{i}, invMass{iM}, invI{iI} {}
	
		index_type GetIndex() const noexcept { return index; }

		float_t invMass; ///< Inverse mass of body.
		float_t invI; ///< Inverse rotational interia of body.

	private:
		index_type index; ///< Index within island of body.
	};
	
	ContactVelocityConstraint() = default;
	ContactVelocityConstraint(const ContactVelocityConstraint& copy) = default;
	ContactVelocityConstraint& operator= (const ContactVelocityConstraint& copy) = default;

	ContactVelocityConstraint(index_type ci, float_t f, float_t r, float_t ts):
		contactIndex{ci}, friction{f}, restitution{r}, tangentSpeed{ts} {}

	/// Gets the count of points added to this object.
	/// @return Value between 0 and MaxManifoldPoints
	/// @sa MaxManifoldPoints.
	/// @sa AddPoint.
	size_type GetPointCount() const noexcept { return pointCount; }
	
	/// Gets the point identified by the given index.
	/// @note Behavior is undefined if the identified point does not exist.
	/// @param index Index of the point to return. This is a value less than returned by GetPointCount().
	/// @return velocity constraint point for the given index.
	/// @sa GetPointCount.
	const VelocityConstraintPoint& GetPoint(size_type index) const
	{
		assert(index < pointCount);
		return points[index];
	}

	/// Gets the point identified by the given index.
	/// @note Behavior is undefined if the identified point does not exist.
	/// @param index Index of the point to return. This is a value less than returned by GetPointCount().
	/// @return velocity constraint point for the given index.
	/// @sa GetPointCount.
	VelocityConstraintPoint& GetPoint(size_type index)
	{
		assert(index < pointCount);
		return points[index];
	}

	void ClearPoints() noexcept { pointCount = 0; }

	/// Adds the given point to this contact velocity constraint object.
	/// @detail Adds up to MaxManifoldPoints points. To find out how many points have already
	///   been added, call GetPointCount().
	/// @param val Velocity constraint point value to add.
	/// @note Behavior is undefined if an attempt is made to add more than MaxManifoldPoints points.
	/// @sa GetPointCount().
	void AddPoint(const VelocityConstraintPoint& val);

	void RemovePoint() noexcept;

	/// Sets this object's K value.
	/// @param value A position constraint dependent value or the zero matrix (Mat22_zero).
	void SetK(const Mat22& value) noexcept;

	/// Gets the "K" value.
	/// @note This value is only valid if SetK had previously been called with a valid value.
	/// @warning Behavior is undefined if called before SetK was called.
	/// @return "K" value previously set.
	Mat22 GetK() const noexcept;

	Mat22 GetNormalMass() const noexcept;

	/// Gets the contact index.
	/// @note This value can only be set via the initializing constructor.
	/// @return Index of the associated contact (the index of the contact that this constraint is for).
	index_type GetContactIndex() const noexcept { return contactIndex; }

	/// Gets the combined friction of the associated contact.
	float_t GetFriction() const noexcept { return friction; }
	
	/// Gets the combined restitution of the associated contact.
	float_t GetRestitution() const noexcept { return restitution; }
	
	/// Gets the tangent speed of the associated contact.
	float_t GetTangentSpeed() const noexcept { return tangentSpeed; }
	
	Vec2 normal; ///< Normal of the world manifold.

	BodyData bodyA; ///< Body A contact velocity constraint data.
	BodyData bodyB; ///< Body B contact velocity constraint data.

private:
	float_t friction; ///< Friction coefficient (4-bytes). Usually in the range of [0,1].
	float_t restitution; ///< Restitution coefficient (4-bytes).
	float_t tangentSpeed; ///< Tangent speed (4-bytes).
	
	index_type contactIndex; ///< Index of the contact that this constraint is for (typically 8-bytes).
	
	// K and normalMass fields are only used for the block solver.
	Mat22 K = Mat22_invalid; ///< Block solver "K" info (only used by block solver, 16-bytes).
	Mat22 normalMass = Mat22_invalid; ///< Block solver "normal mass" info (only used by block solver, 16-bytes).

	VelocityConstraintPoint points[MaxManifoldPoints]; ///< Velocity constraint points array (at least 72-bytes).
	size_type pointCount = 0; ///< Point count (at least 1-byte).
};

inline void ContactVelocityConstraint::AddPoint(const VelocityConstraintPoint& val)
{
	assert(pointCount < MaxManifoldPoints);
	points[pointCount] = val;
	++pointCount;
}

inline void ContactVelocityConstraint::RemovePoint() noexcept
{
	assert(pointCount > 0);
	--pointCount;
}

inline void ContactVelocityConstraint::SetK(const Mat22& value) noexcept
{
	assert(IsValid(value));
	K = value;
	normalMass = Invert(value);
}

inline Mat22 ContactVelocityConstraint::GetK() const noexcept
{
	assert(IsValid(K));
	return K;
}

inline Mat22 ContactVelocityConstraint::GetNormalMass() const noexcept
{
	assert(IsValid(normalMass));
	return normalMass;
}

/// Contact Position Constraint.
/// @note This structure is at least 104-bytes large.
struct ContactPositionConstraint
{
	using size_type = std::remove_const<decltype(MaxManifoldPoints)>::type;
	
	/// Position constraint body data.
	struct BodyData
	{
		using index_type = std::remove_const<decltype(MaxBodies)>::type;
		
		BodyData() noexcept = default;
	
		constexpr BodyData(index_type i, float_t iM, float_t iI, Vec2 lc) noexcept: index{i}, invMass{iM}, invI{iI}, localCenter{lc} {}
		
		index_type index; ///< Index within island of the associated body (2-bytes).
		float_t invMass; ///< Inverse mass of associated body (a non-negative value, 4-bytes).
		float_t invI; ///< Inverse rotational inertia about the center of mass of the associated body (a non-negative value, 4-bytes).
		Vec2 localCenter; ///< Local center of the associated body's sweep (8-bytes).
	};
	
	ContactPositionConstraint() = default;
	
	ContactPositionConstraint(const Manifold& m, const BodyData& bA, float_t rA, const BodyData& bB, float_t rB):
		manifold{m}, bodyA{bA}, radiusA{rA}, bodyB{bB}, radiusB{rB} {}

	Manifold manifold; ///< Copy of contact's manifold (at least 59-bytes).
	
	BodyData bodyA; ///< Body A data (at least 18-bytes).
	
	float_t radiusA; ///< "Radius" distance from the associated shape of fixture A (4-bytes).

	BodyData bodyB; ///< Body A data (at least 18-bytes).
	
	float_t radiusB; ///< "Radius" distance from the associated shape of fixture B (4-bytes).
};
	
/// Contact Solver.
class ContactSolver
{
public:
	/// Minimum separation for position constraints.
	static constexpr auto MinSeparationThreshold = BOX2D_MAGIC(-LinearSlop * 3);

	/// Minimum time of impact separation for TOI position constraints.
	static constexpr auto MinToiSeparation = BOX2D_MAGIC(-LinearSlop * float_t{3} / float_t{2}); // aka -LinearSlop * 1.5

	/// Initializing constructor.
	/// @param positions Array of positions, one for every body referenced by a contact.
	/// @param velocities Array of velocities, for every body referenced by a contact.
	/// @param count Count of contacts.
	/// @param positionConstraints Array of position-constraints (1 per contact).
	/// @param velocityConstraints Array of velocity-constraints (1 per contact).
	ContactSolver(Position* positions, Velocity* velocities,
				  contact_count_t count,
				  ContactPositionConstraint* positionConstraints,
				  ContactVelocityConstraint* velocityConstraints) noexcept :
		m_positions{positions},
		m_velocities{velocities},
		m_count{count},
		m_positionConstraints{positionConstraints},
		m_velocityConstraints{velocityConstraints}
	{
	}
	
	~ContactSolver() = default;

	ContactSolver() = delete;
	ContactSolver(const ContactSolver& copy) = delete;

	/// Updates velocity constraints.
	/// @detail
	/// Updates the position dependent portions of the velocity constraints with the
	/// information from the current position constraints.
	/// @post Velocity constraints will have their "normal" field set to the world manifold normal for them.
	/// @post Velocity constraints will have their constraint points updated.
	void UpdateVelocityConstraints();

	void WarmStart();

	/// "Solves" the velocity constraints.
	/// @detail Updates the velocities and velocity constraint points' normal and tangent impulses.
	void SolveVelocityConstraints();

	/// Solves position constraints.
	/// @detail This updates positions (and nothing else).
	/// @return true if the minimum separation is above the minimum separation threshold, false otherwise.
	/// @sa MinSeparationThreshold.
	bool SolvePositionConstraints();
	
	/// Solves TOI position constraints.
	/// @detail Sequential position solver for TOI-based position constraints.
	///   This only updates positions for the bodies identified by the given indexes (and nothing else).
	/// @param indexA Index within the island of body A.
	/// @param indexB Index within the island of body B.
	/// @return true if the minimum separation is above the minimum TOI separation value, false otherwise.
	bool SolveTOIPositionConstraints(island_count_t indexA, island_count_t indexB);

private:

	Position* const m_positions;
	Velocity* const m_velocities;
	
	const contact_count_t m_count; ///< Count of elements (contacts) in the contact position-constraint and velocity-constraint arrays.
	ContactPositionConstraint* const m_positionConstraints; ///< Array of position-constraints (1 per contact).
	ContactVelocityConstraint* const m_velocityConstraints; ///< Array of velocity-constraints (1 per contact).
};

} // namespace box2d

#endif

