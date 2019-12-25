#pragma once

#include <tuple>

#include <osg/Vec3d>
#include <osg/Matrix>
#include <osg/Quat>

typedef double PhysicsFloat;
typedef double TimeType;
typedef osg::Vec3d PhysicsVec3D;
typedef osg::Matrixd PhysicsMatrix;
typedef osg::Quat PhysicsQuat;
typedef std::tuple<size_t, size_t, size_t> FaceIndices;