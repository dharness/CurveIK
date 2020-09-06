#pragma once

#include "CoreMinimal.h"
#include "CurveCache.h"
#include "IKCurve.h"

class IKCurveLine : public IKCurve
{

public:

	IKCurveLine(FVector StartPoint, FVector EndPoint, FVector DefaultNormalDir, float Length)
		: StartPoint(StartPoint)
		, EndPoint(EndPoint)
		, DefaultNormalDir (DefaultNormalDir)
		, Length(Length)
	{
		Direction = (EndPoint - StartPoint).GetSafeNormal();
	};

	FVector StartPoint;
	FVector EndPoint;
	FVector DefaultNormalDir;
	float Length;

	FVector Evaluate(float T) const override;
	FVector EvaluateDerivative(float T) const override;
	FVector EvaluateNormal(float T) const override;
	void EvaluateMany(int32 NumPoints) override;
	FCurvePoint Approximate(float TargetArcLength) override;

private:
	FVector Direction;
};
