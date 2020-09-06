#pragma once

#include "CoreMinimal.h"
#include "CurveCache.h"
#include "IKCurve.h"

class IKCurveBezier : public IKCurve
{
	
public:
	IKCurveBezier()
		: A(FVector::ZeroVector)
		, B(FVector::ZeroVector)
		, C(FVector::ZeroVector)
	{
	}

	IKCurveBezier(FVector const A, FVector const B, FVector const C)
		: A(A)
		, B(B)
		, C(C)
	{
	}
	
	FVector A;
	FVector B;
	FVector C;

	FVector Evaluate(float T) const override;
	FVector EvaluateDerivative(float T) const override;
	FVector EvaluateNormal(float T) const override;
	void EvaluateMany(int32 NumPoints) override;
	FCurvePoint Approximate(float TargetArcLength) override;
};
