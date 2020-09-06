#include "IKCurves/IKCurveLine.h"

FVector IKCurveLine::Evaluate(float T) const
{
	return Direction * Length * T;
}

FVector IKCurveLine::EvaluateDerivative(float T) const
{
	return Direction;
}

FVector IKCurveLine::EvaluateNormal(float T) const
{
	return DefaultNormalDir;
}

void IKCurveLine::EvaluateMany(int32 NumPoints)
{
	const float MinT = 0;
	const float MaxT = 1;
	const float StepSize = MaxT / (NumPoints - 1);
	float T = MinT;
	ArcLength = 0;

	CurveCache = FCurveIK_CurveCache();

	FVector PrevPoint = Evaluate(T);
	CurveCache.Add(ArcLength, PrevPoint, T);

	for (int i = 1; i < NumPoints; i++)
	{
		T += StepSize;
		const FVector Point = Evaluate(T);
		ArcLength += FVector::Dist(PrevPoint, Point);
		PrevPoint = Point;

		CurveCache.Add(ArcLength, Point, T);
	}
}

FCurvePoint IKCurveLine::Approximate(float TargetArcLength)
{
	FCurvePoint CurvePoint;
	float const T = TargetArcLength / Length;
	UE_LOG(LogTemp, Warning, TEXT("TargetArcLength: %f"), TargetArcLength);
	UE_LOG(LogTemp, Warning, TEXT("Length: %f"), Length);
	UE_LOG(LogTemp, Warning, TEXT("T: %f"), T);
	CurvePoint.T = T;
	CurvePoint.ArcLength = TargetArcLength;
	CurvePoint.Point = Evaluate(T);
	CurvePoint.Normal = EvaluateNormal(T);
	CurvePoint.Tangent = EvaluateDerivative(T);
	return CurvePoint;
}
