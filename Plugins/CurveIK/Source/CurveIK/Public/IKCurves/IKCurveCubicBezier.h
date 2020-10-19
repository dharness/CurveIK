#pragma once
#pragma once

#include "CoreMinimal.h"
#include "CurveCache.h"
#include "IKCurve.h"

/**
 * Implements a quadratic bezier curve type. Handles and points are described by the control polygon
 * defined by points A, B, and C
 *
 */

class IKCurveCubicBezier : public IKCurve
{
public:
	FCurveIK_CurveCache CurveCache;

	IKCurveCubicBezier()
		: A(FVector::ZeroVector)
		, B(FVector::ZeroVector)
		, C(FVector::ZeroVector)
		, D(FVector::ZeroVector)
	{
	}

	IKCurveCubicBezier(FVector const A, FVector const B, FVector const C)
		: A(A)
		, B(B)
		, C(C)
	{
		CurveType = IK_QuadraticBezier;
	}

	IKCurveCubicBezier(FVector const A, FVector const B, FVector const C, FVector const D)
		: A(A)
		, B(B)
		, C(C)
		, D(D)
	{
		CurveType = IK_CubicBezier;
	}

	// IKCurve base class
	FVector Evaluate(float T) const override;
	FVector EvaluateDerivative(float T) const override;
	FVector EvaluateNormal(float T) const override;
	FCurvePoint Approximate(float TargetArcLength) override;
	// End of IKCurve base class

	/*
	 * Evaluates some number of points on the curve and caches their value.
	 * This method must be called before using IKCurveCubicBezier::Approximate
	 */
	void EvaluateMany(int32 NumPoints);

	/*
	 * Iteratively searches the space of possible curves that extend from P1 to P2
	 * while varying the height until a curve with the proper arc-length is found.
	 *
	 * @return The Bezier curve with the closest arc-length to the target within the allowable tolerance
	 */
	static IKCurveCubicBezier* FindCurve(FVector P1, FVector P2, FVector HandleDir, float HandleWeight,
	                                                         float TargetArcLength, int MaxIterations,
	                                                         float CurveFitTolerance, int NumPoints,
	                                                         TArray<FVector>& ControlPoints, float HandleAngle,
	                                                         EIKCurveTypes CurveType);

private:
	FVector A;
	FVector B;
	FVector C;
	FVector D;

	EIKCurveTypes CurveType = IK_QuadraticBezier;

	/*
	 * Combines start position, direction, and distance to produce a vector describing a handle location
	 * in component space.
	 */
	static FVector GetHandleLocation(FVector HandleStart, FVector HandleDir, float HandleHeight);

	/*
	 * Approximates the handle height for a given arc length.
	 */
	static float GetHandleHeight(FVector P1, FVector P2, float HandleWeight, float ArcLength);
};
