// SimulatorBackend.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "SimulatorBackend.h"
#include "SimulatorTypes.h"
#include "PolyhedralGeometry.h"
#include "PhysicsObject.h"

#include <iostream>

void test()
{
	PolyhedralGeometry testGeometry({
		PhysicsVec3D(-1.0, 1.0, 0.0),
		PhysicsVec3D(1.0, 1.0, 0.0),
		PhysicsVec3D(1.0, -1.0, 0.0),
		PhysicsVec3D(-1.0, -1.0, 0.0),
		PhysicsVec3D(0.0, 0.0, 1.0)
	},
	{
		{0, 1, 2},
		{2, 3, 0},
		{0, 1, 4},
		{1, 2, 4},
		{2, 3, 4},
		{3, 0, 4}
	});

	auto bBox = testGeometry.computeBoundingBox();

	std::cout << "Bounding box: {" << bBox.xMin() << ", " << bBox.yMin() << ", " << bBox.zMin() << "} - {"
		<< bBox.xMax() << ", " << bBox.yMax() << ", " << bBox.zMax() << "}" << std::endl;
	
	auto faceNormals = testGeometry.computeFaceNormals();

	std::cout << "Face normals: " << std::endl;
	for (const PhysicsVec3D& thisPoint : faceNormals)
	{
		std::cout << thisPoint.x() << ", " << thisPoint.y() << ", " << thisPoint.z() << " | ";
	}
	std::cout << std::endl
			<< "Volume: " << testGeometry.getVolume() << std::endl;

	PhysicsObject testObj(testGeometry, PhysicsVec3D(0.0, 0.0, 10.0), 1.0, PhysicsQuat(),
		PhysicsVec3D(), PhysicsVec3D(0.0, 0.0, 1.0) * (2 * 3.1415926535));

	std::vector<PhysicsVec3D> testForces = {
		PhysicsVec3D(0.0, 0.0, -1.0),
		PhysicsVec3D(1.0, 0.0, 1.0)
	};

	for(unsigned i = 0; i < 101; i++)
	{
		testObj.update(testForces, 0.01);

		const std::vector<PhysicsVec3D>& posRef = testObj.getRelativePointPositions();
		PhysicsVec3D objPos = testObj.getPosition();

		for(const PhysicsVec3D& thisPoint : posRef)
		{
			std::cout << objPos.x() + thisPoint.x() << ", " << objPos.y() + thisPoint.y() << ", " << objPos.z() + thisPoint.z() << " | ";
		}

		std::cout << std::endl;
	}
}
