#pragma once

#include "SimulatorTypes.h"

inline PhysicsQuat makeQuatXYZIntrinsic(PhysicsFloat x, PhysicsFloat y, PhysicsFloat z)
{
	PhysicsQuat rotQuat = PhysicsQuat(x, PhysicsVec3D(1.0, 0.0, 0.0));
	rotQuat = PhysicsQuat(y, PhysicsVec3D(0.0, 1.0, 0.0)) * rotQuat;
	rotQuat = PhysicsQuat(z, PhysicsVec3D(0.0, 0.0, 1.0)) * rotQuat;

	return rotQuat;
}

inline PhysicsQuat axialVectorToQuat(const PhysicsVec3D& vec)
{
	return PhysicsQuat(vec.length(), vec);
}

inline PhysicsFloat scalarProjection(const PhysicsVec3D& vec, const PhysicsVec3D& projBasis)
{
	return (projBasis * vec) / projBasis.length();
}

// inline PhysicsQuat scaleQuatAngle(const PhysicsQuat& srcQuat, PhysicsFloat scale)
// {
// 	PhysicsFloat srcAngle;
// 	PhysicsVec3D srcAxis;
// 	
// 	srcQuat.getRotate(srcAngle, srcAxis);
// 	return PhysicsQuat()
// }
