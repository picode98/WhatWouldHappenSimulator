#pragma once

#include <vector>
#include <array>
#include <deque>
#include <cassert>

#include <osg/Matrix>

#include "SimulatorTypes.h"
#include "VectorUtils.h"

class PolyhedralGeometry
{
private:
	std::vector<PhysicsVec3D> geomPoints;
	std::vector<FaceIndices> geomFaces;
	std::vector<std::vector<size_t>> pointsToFaces;
	// std::map<std::pair<size_t, size_t>, size_t> edgesToFaces;
	
	bool boundingBoxValid = false,
		faceNormalsValid = false;
	GeomBoundingBox boundingBox;
	size_t sampleBoundingPoint = 0;

	std::vector<PhysicsVec3D> faceNormals;

	template<typename ElemType>
	static std::vector<ElemType> sortedVectorIntersect(const std::vector<ElemType>& firstVec, const std::vector<ElemType>& secondVec)
	{
		std::vector<ElemType> result;
		
		size_t secondVecIndex = 0;

		for(const ElemType& thisFirstVecElem : firstVec)
		{
			if (result.empty() || thisFirstVecElem != result.back())
			{
				for (; secondVecIndex < secondVec.size(); secondVecIndex++)
				{
					if (thisFirstVecElem == secondVec[secondVecIndex])
					{
						result.push_back(thisFirstVecElem);
						break;
					}

					if (thisFirstVecElem < secondVec[secondVecIndex])
					{
						break;
					}
				}
			}
		}

		return result;
	}
	
public:
	PolyhedralGeometry(const std::vector<PhysicsVec3D>& initGeomPoints, const std::vector<FaceIndices> initGeomFaces):
		geomPoints(initGeomPoints),
		geomFaces(initGeomFaces),
		faceNormals(initGeomFaces.size())
	{
		for (size_t i = 0; i < geomFaces.size(); i++)
		{
			const FaceIndices& thisFace = geomFaces[i];
			for(size_t thisPoint : thisFace)
			{
				if((thisPoint + 1) > pointsToFaces.size())
				{
					pointsToFaces.resize(thisPoint + 1);
				}

				pointsToFaces[thisPoint].push_back(i);
			}
			
			// edgesToFaces.insert({ { thisFace[0], thisFace[1] }, i });
		}
	}

	void translate(const PhysicsVec3D& offset)
	{
		for (PhysicsVec3D& thisPoint : geomPoints)
		{
			thisPoint += offset;
		}

		if (boundingBoxValid)
		{
			boundingBox.set(boundingBox.xMin() + offset.x(), boundingBox.yMin() + offset.y(), boundingBox.zMin() + offset.z(),
				boundingBox.xMax() + offset.x(), boundingBox.yMax() + offset.y(), boundingBox.zMax() + offset.z());
		}
	}

	void rotate(const PhysicsQuat& rotQuat, const PhysicsVec3D& origin)
	{
		PhysicsMatrix rotMatrix = PhysicsMatrix();
		rotMatrix.makeRotate(rotQuat);
		
		for (PhysicsVec3D& thisPoint : geomPoints)
		{
			thisPoint = rotMatrix.postMult(thisPoint - origin) + origin;
		}

		if(faceNormalsValid)
		{
			for (PhysicsVec3D& thisFaceNormal : faceNormals)
			{
				thisFaceNormal = rotMatrix.postMult(thisFaceNormal);
			}
		}

		boundingBoxValid = false;
	}

	const std::vector<PhysicsVec3D>& getPoints() const
	{
		return this->geomPoints;
	}

	const std::vector<FaceIndices>& getFaces() const
	{
		return this->geomFaces;
	}

	GeomBoundingBox computeBoundingBox()
	{
		if (!this->boundingBoxValid)
		{
			PhysicsVec3D min = this->geomPoints[0], max = min;
			this->sampleBoundingPoint = 0;

			for(size_t i = 1; i < this->geomPoints.size(); i++)
			{
				const PhysicsVec3D& thisPt = this->geomPoints[i];

				if(thisPt.x() > max.x())
				{
					max.x() = thisPt.x();
					this->sampleBoundingPoint = i;
				}
				else if(thisPt.x() < min.x())
				{
					min.x() = thisPt.x();
				}

				if (thisPt.y() > max.y())
				{
					max.y() = thisPt.y();
				}
				else if (thisPt.y() < min.y())
				{
					min.y() = thisPt.y();
				}

				if (thisPt.z() > max.z())
				{
					max.z() = thisPt.z();
				}
				else if (thisPt.z() < min.z())
				{
					min.z() = thisPt.z();
				}
			}

			this->boundingBox.set(min, max);
			this->boundingBoxValid = true;
		}

		return this->boundingBox;
	}

	struct FaceNormalFrame
	{
		size_t faceIndex,
			leftPointIndex,
			rightPointIndex;
	};

	std::vector<PhysicsVec3D> computeFaceNormals()
	{
		if (!this->faceNormalsValid)
		{
			this->computeBoundingBox();

			// std::stack<FaceNormalFrame> faceStack;

			std::deque<FaceNormalFrame> faceQueue;
			std::vector<bool> facesVisited(this->geomFaces.size());

			for (size_t thisFaceIndex : this->pointsToFaces[this->sampleBoundingPoint])
			{
				const FaceIndices& thisFacePoints = this->geomFaces[thisFaceIndex];

				size_t firstOtherPoint = (thisFacePoints[0] == this->sampleBoundingPoint ? thisFacePoints[1] : thisFacePoints[0]),
					secondOtherPoint = (thisFacePoints[2] == this->sampleBoundingPoint ? thisFacePoints[1] : thisFacePoints[2]);

				// LR x RO = (R - L) x (O - R)
				PhysicsVec3D firstCrossProduct = (this->geomPoints[this->sampleBoundingPoint] - this->geomPoints[firstOtherPoint]) ^
					(this->geomPoints[secondOtherPoint] - this->geomPoints[this->sampleBoundingPoint]);

				if (firstCrossProduct.x() != 0.0)
				{
					// First left and right points should be chosen such that the normal vector has a positive x component.
					FaceNormalFrame initialFrame = {
						thisFaceIndex,
						(firstCrossProduct.x() > 0.0 ? firstOtherPoint : secondOtherPoint),
						this->sampleBoundingPoint
					};

					faceQueue.push_back(initialFrame);

					break;
				}
			}

			assert(!faceQueue.empty());

			// Normal 

			//
			// for(size_t thisFace : this->pointsToFaces[this->sampleBoundingPoint])
			// {
			// 	faceQueue.push_back({ thisFace, this->sampleBoundingPoint });
			// 	facesVisited[thisFace] = true;
			// }
			//
			while (!faceQueue.empty())
			{
				const FaceNormalFrame& thisItem = faceQueue.front();
				const FaceIndices& thisFacePoints = this->geomFaces[thisItem.faceIndex];
				size_t otherPointIndex;

				if (thisFacePoints[0] != thisItem.leftPointIndex && thisFacePoints[0] != thisItem.rightPointIndex)
				{
					otherPointIndex = thisFacePoints[0];
				}
				else if (thisFacePoints[1] != thisItem.leftPointIndex && thisFacePoints[1] != thisItem.rightPointIndex)
				{
					otherPointIndex = thisFacePoints[1];
				}
				else
				{
					otherPointIndex = thisFacePoints[2];
				}

				this->faceNormals[thisItem.faceIndex] =
					(this->geomPoints[thisItem.rightPointIndex] - this->geomPoints[thisItem.leftPointIndex]) ^
					(this->geomPoints[otherPointIndex] - this->geomPoints[thisItem.rightPointIndex]);
				facesVisited[thisItem.faceIndex] = true;

				std::vector<size_t> leftCommonFaces =
					sortedVectorIntersect(this->pointsToFaces[thisItem.leftPointIndex], this->pointsToFaces[otherPointIndex]),
					rightCommonFaces =
					sortedVectorIntersect(this->pointsToFaces[thisItem.rightPointIndex], this->pointsToFaces[otherPointIndex]);

				assert(leftCommonFaces.size() == 2);
				assert(rightCommonFaces.size() == 2);

				size_t leftFaceIndex = (leftCommonFaces[0] == thisItem.faceIndex ? leftCommonFaces[1] : leftCommonFaces[0]),
					rightFaceIndex = (rightCommonFaces[0] == thisItem.faceIndex ? rightCommonFaces[1] : rightCommonFaces[0]);

				if (!facesVisited[leftFaceIndex])
				{
					faceQueue.push_back({
						leftFaceIndex,
						thisItem.leftPointIndex,
						otherPointIndex
						});
				}

				if (!facesVisited[rightFaceIndex])
				{
					faceQueue.push_back({
						rightFaceIndex,
						otherPointIndex,
						thisItem.rightPointIndex
						});
				}

				faceQueue.pop_front();
			}

			this->faceNormalsValid = true;
			// 	const std::pair<size_t, size_t>& thisItem = faceQueue.front();
			// 	const FaceIndices& thisFacePoints = this->geomFaces[thisItem.first];
			// 	size_t firstOtherPoint = (thisFacePoints[0] == thisItem.second ? thisFacePoints[1] : thisFacePoints[0]),
			// 		secondOtherPoint = (thisFacePoints[2] == thisItem.second ? thisFacePoints[1] : thisFacePoints[2]);
			//
			// 	PhysicsFloat dot = this->geomPoints[firstOtherPoint] * this->geomPoints[secondOtherPoint];
			// 	faceQueue.pop_front();
			// }
		}

		return this->faceNormals;
	}

	PhysicsFloat getVolume()
	{
		this->computeFaceNormals();

		double totalVolume = 0.0;

		for(size_t i = 0; i < this->geomFaces.size(); i++)
		{
			PhysicsFloat faceArea = 0.5 * this->faceNormals[i].length(),
				tetrahedronSignedHeight = scalarProjection(this->geomPoints[this->geomFaces[i][0]], this->faceNormals[i]);

			totalVolume += (1.0 / 3.0) * faceArea * tetrahedronSignedHeight;
		}

		return totalVolume;
	}
};
