#include "IKCurves/IKCurveLine.h"


IKCurveLine* IKCurveLine::FindCurve(FVector P1, FVector P2, FVector HandleDir, float TargetArcLength)
{
	FVector LineDir = P2 - P1;
	return new IKCurveLine(P1, P2, HandleDir, TargetArcLength);
}


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

FCurvePoint IKCurveLine::Approximate(float TargetArcLength)
{
	FCurvePoint CurvePoint;
	float const T = TargetArcLength / Length;
	CurvePoint.T = T;
	CurvePoint.ArcLength = TargetArcLength;
	CurvePoint.Point = Evaluate(T);
	CurvePoint.Normal = EvaluateNormal(T);
	CurvePoint.Tangent = EvaluateDerivative(T);
	return CurvePoint;
}