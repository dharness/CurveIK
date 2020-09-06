#pragma once

#include "CoreMinimal.h"
#include "CurveCache.h"

class IKCurve
{
public:
	FCurveIK_CurveCache CurveCache;

	float ArcLength = 0;

	IKCurve() {};
	virtual ~IKCurve() {};
	
	virtual FVector Evaluate(float T) const = 0;

	virtual FVector EvaluateDerivative(float T) const = 0;

	virtual FVector EvaluateNormal(float T) const = 0;

	virtual void EvaluateMany(int32 NumPoints) = 0;

	virtual FCurvePoint Approximate(float TargetArcLength) = 0;
};

