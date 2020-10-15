#pragma once

#include "CoreMinimal.h"
#include "CurveCache.h"

UENUM(BlueprintType)
enum EIKCurveTypes
{
	IK_QuadraticBezier UMETA(DisplayName = "Quadratic Bezier"),
	IK_CubicBezier UMETA(DisplayName = "Cubic Bezier"),
};

/*
 * Abstract base class representing all the required methods for a curve to be useable in the IK system.
 * To add a new curve type, extend this class.
 */
class IKCurve
{
public:
	float ArcLength = 0;

	IKCurve() {};
	virtual ~IKCurve() {};

	/**
	 * Provide the 3D point on the curve parameterized by  T
	 * 
	 * @param T A value in [0, 1] where 0 is the start of the curve and 1 is the end
	 */
	virtual FVector Evaluate(float T) const = 0;

	/**
	 * Provide the tangent vector of the curve parameterized by  T
	 *
	 * @param T A value in [0, 1] where 0 is the start of the curve and 1 is the end
	 */
	virtual FVector EvaluateDerivative(float T) const = 0;

	/**
	 * Provide the normal vector of the curve parameterized by  T
	 *
	 * @param T A value in [0, 1] where 0 is the start of the curve and 1 is the end
	 */
	virtual FVector EvaluateNormal(float T) const = 0;

	/**
	 * Approximates the point on a curve for a given arc-length. In performance sensitive situations,
	 * this method should be used over Evaluate. Negative arc-lengths or arc-lengths longer than the
	 * defined curve will return the points at the start or end of the curve respectively.
	 * The actual arc-length of the returned point may vary.
	 *
	 * @param TargetArcLength The arc-length for which we want to retrieve a point on the curve.
	 */
	virtual FCurvePoint Approximate(float TargetArcLength) = 0;
};

