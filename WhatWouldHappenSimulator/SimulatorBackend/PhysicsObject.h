#pragma once
#include "SimulatorTypes.h"
#include "PolyhedralGeometry.h"
#include "VectorUtils.h"

#include <osg/Geometry>

class PhysicsObject
{
private:
	PhysicsVec3D position = PhysicsVec3D(),
	velocity = PhysicsVec3D(),
	centroid = PhysicsVec3D();
	// PhysicsQuat rotation = PhysicsQuat(),
	PhysicsVec3D rotationalVelocity = PhysicsVec3D();

	const PolyhedralGeometry& geometryRef;
	PolyhedralGeometry rotatedGeometry;

	PhysicsFloat mass = 0.0;
public:
	PhysicsObject(const PolyhedralGeometry& geometryRef, const PhysicsVec3D& initialPosition, const PhysicsFloat initialMass, const PhysicsQuat& initialRotation,
		const PhysicsVec3D& initialVelocity = PhysicsVec3D(), const PhysicsVec3D& initialRotVelocity = PhysicsVec3D()):
		geometryRef(geometryRef),
		rotatedGeometry(geometryRef),
		// rotation(rotation),
		position(initialPosition),
		mass(initialMass),
		velocity(initialVelocity),
		rotationalVelocity(initialRotVelocity)
	{
		this->rotate(initialRotation);
	}

	void setPosition(const PhysicsVec3D& newPosition)
	{
		this->position = newPosition;
	}

	PhysicsVec3D getPosition() const
	{
		return this->position;
	}

	void rotate(const PhysicsQuat& rotation)
	{
		rotatedGeometry.rotate(rotation, this->centroid);
		// this->rotation = rotation * this->rotation;
	}

	void setRotation(const PhysicsQuat& rotation)
	{
		rotatedGeometry = geometryRef;
		this->rotate(rotation);
		// this->rotation = rotation;
	}

	/*
	PhysicsQuat getRotation() const
	{
		return this->rotation;
	}
	*/

	const std::vector<PhysicsVec3D>& getRelativePointPositions() const
	{
		return (this->rotatedGeometry).getPoints();
	}

	void update(const std::vector<PhysicsVec3D>& forces, TimeType time)
	{
		PhysicsVec3D acceleration = PhysicsVec3D();

		for (const PhysicsVec3D& force : forces)
		{
			acceleration += force;
		}
		acceleration /= this->mass;

		this->velocity += (acceleration * time);
		this->position += (this->velocity * time);

		// TODO: torques and angular acceleration
		this->rotate(axialVectorToQuat(this->rotationalVelocity * time));
	}
};
