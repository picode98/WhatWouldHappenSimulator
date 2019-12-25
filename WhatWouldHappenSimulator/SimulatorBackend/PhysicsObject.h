#pragma once
#include "SimulatorTypes.h"
#include "PolyhedralGeometry.h"

class PhysicsObject
{
private:
	PhysicsVec3D position = PhysicsVec3D(),
	velocity = PhysicsVec3D();
	PhysicsQuat rotation = PhysicsQuat();

	const PolyhedralGeometry& geometryRef;
	PolyhedralGeometry rotatedGeometry;
public:
	PhysicsObject(const PolyhedralGeometry& geometryRef, const PhysicsVec3D& position, const PhysicsQuat& rotation):
		geometryRef(geometryRef),
		rotatedGeometry(geometryRef),
		rotation(rotation),
		position(position)
	{
		rotatedGeometry.rotate(rotation);
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
		rotatedGeometry.rotate(rotation);
		this->rotation = rotation * this->rotation;
	}

	void setRotation(const PhysicsQuat& rotation)
	{
		rotatedGeometry = geometryRef;
		rotatedGeometry.rotate(rotation);
		this->rotation = rotation;
	}

	PhysicsQuat getRotation() const
	{
		return this->rotation;
	}

	const std::vector<PhysicsVec3D>& getRelativePointPositions() const
	{
		return (this->rotatedGeometry).getPoints();
	}
};