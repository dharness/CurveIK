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
		, DefaultNormalDir(DefaultNormalDir)
		, Length(Length)
	{
		Direction = (EndPoint - StartPoint).GetSafeNormal();
	};

	// IKCurve base class
	FVector Evaluate(float T) const override;
	FVector EvaluateDerivative(float T) const override;
	FVector EvaluateNormal(float T) const override;
	FCurvePoint Approximate(float TargetArcLength) override;
	// End of IKCurve base class

	/*
	 * Provides a curve object described by the given parameters
	 */
	static IKCurveLine* FindCurve(FVector P1, FVector P2, FVector HandleDir, float TargetArcLength);

private:
	FVector Direction;
	FVector StartPoint;
	FVector EndPoint;
	FVector DefaultNormalDir;
	float Length;
};
