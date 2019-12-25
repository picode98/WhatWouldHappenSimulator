#pragma once

#include <vector>

#include <osg/Matrix>

#include "SimulatorTypes.h"

class PolyhedralGeometry
{
private:
	std::vector<PhysicsVec3D> geomPoints;
	std::vector<FaceIndices> geomFaces;
	
public:
	PolyhedralGeometry(const std::vector<PhysicsVec3D>& initGeomPoints, const std::vector<FaceIndices> initGeomFaces):
		geomPoints(initGeomPoints),
		geomFaces(initGeomFaces)
	{}

	void translate(const PhysicsVec3D& offset)
	{
		for (PhysicsVec3D& thisPoint : geomPoints)
		{
			thisPoint += offset;
		}
	}

	void rotate(const PhysicsQuat& rotQuat)
	{
		PhysicsMatrix rotMatrix = PhysicsMatrix();
		rotMatrix.makeRotate(rotQuat);
		
		for (PhysicsVec3D& thisPoint : geomPoints)
		{
			thisPoint = rotMatrix.postMult(thisPoint);
		}
	}

	const std::vector<PhysicsVec3D>& getPoints() const
	{
		return this->geomPoints;
	}

	const std::vector<FaceIndices>& getFaces() const
	{
		return this->geomFaces;
	}
};