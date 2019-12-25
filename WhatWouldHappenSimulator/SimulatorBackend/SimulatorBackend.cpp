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

	PhysicsObject testObj(testGeometry, PhysicsVec3D(0.0, 0.0, 0.0), PhysicsQuat());

	for(unsigned i = 0; i < 101; i++)
	{
		testObj.rotate(PhysicsQuat(3.1415 / 50.0, PhysicsVec3D(0.0, 0.0, 1.0)));

		const std::vector<PhysicsVec3D>& posRef = testObj.getRelativePointPositions();
		PhysicsVec3D objPos = testObj.getPosition();

		for(const PhysicsVec3D& thisPoint : posRef)
		{
			std::cout << objPos.x() + thisPoint.x() << ", " << objPos.y() + thisPoint.y() << ", " << objPos.z() + thisPoint.z() << " | ";
		}

		std::cout << std::endl;
	}
}
