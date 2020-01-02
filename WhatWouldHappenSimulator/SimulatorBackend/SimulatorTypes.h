#pragma once

#include <array>

#include <osg/Vec3d>
#include <osg/Matrix>
#include <osg/Quat>
#include <osg/BoundingBox>

typedef double PhysicsFloat;
typedef double TimeType;
typedef osg::Vec3d PhysicsVec3D;
typedef osg::Matrixd PhysicsMatrix;
typedef osg::Quat PhysicsQuat;
typedef std::array<size_t, 3> FaceIndices;
typedef osg::BoundingBox GeomBoundingBox;