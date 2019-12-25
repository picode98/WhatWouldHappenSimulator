#pragma once

#include "SimulatorTypes.h"

inline PhysicsQuat makeQuatXYZIntrinsic(PhysicsFloat x, PhysicsFloat y, PhysicsFloat z)
{
	PhysicsQuat rotQuat = PhysicsQuat(x, PhysicsVec3D(1.0, 0.0, 0.0));
	rotQuat = PhysicsQuat(y, PhysicsVec3D(0.0, 1.0, 0.0)) * rotQuat;
	rotQuat = PhysicsQuat(z, PhysicsVec3D(0.0, 0.0, 1.0)) * rotQuat;

	return rotQuat;
}
