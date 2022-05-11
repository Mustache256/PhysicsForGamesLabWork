#include "Utility.h"

namespace PFG
{
	float DistanceToPlane(const glm::vec3& normal, const glm::vec3& centerPoint, const glm::vec3& q)
	{
		float d = glm::dot((centerPoint - q), normal);
		return d;
	}

	bool MovingSphereToPlaneCollision(const glm::vec3& normal, const glm::vec3& centerPoint1, const glm::vec3& centerPoint2, const glm::vec3& q, float radius, glm::vec3& contactPoint)
	{
		float t;

		float distance0 = DistanceToPlane(normal, centerPoint1, q);
		float distance1 = DistanceToPlane(normal, centerPoint2, q);

		if (glm::abs(distance0) <= radius)
		{
			contactPoint = centerPoint1;
			t = 0.0f;
			return true;
		}
		if (distance0 > radius && distance1 < radius)
		{
			t = (distance0 - radius) / (distance0 - distance1);
			contactPoint = (1 - t) * centerPoint1 + t * centerPoint2;
			return true;
		}
		return false;
	}


	bool SphereToSphereCollision(const glm::vec3& centerPoint1, const glm::vec3 centerPoint2, float radius1, float radius2, glm::vec3& contactPoint)
	{
		float distance = glm::length(centerPoint1 - centerPoint2);
		glm::vec3 normal;

		if (distance <= (radius1 + radius2))
		{
			normal = glm::normalize(centerPoint1 - centerPoint2);
			contactPoint = radius1 * normal;

			return true;
		}
		return false;
	}
}