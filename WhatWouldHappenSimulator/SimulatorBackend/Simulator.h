#pragma once
#include "SimulatorTypes.h"
#include "PhysicsObject.h"
#include <osg/Shape>

class Simulator
{
private:
	// osg::KdTree tree;
	// PhysicsObject obj = PhysicsObject();

public:
	void advanceTime(TimeType time)
	{
		//tree.addLine(0, 5);
		//tree.addLine(7, 13);
		//tree.addLine(4, 6);

		osg::ref_ptr<osg::Box> testBox = new osg::Box(osg::Vec3(1.0, 2.0, 3.0), 4.0);
		// osg::
	}
};
