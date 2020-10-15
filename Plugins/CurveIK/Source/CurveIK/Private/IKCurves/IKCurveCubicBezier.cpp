#include "IKCurves/IKCurveCubicBezier.h"


FVector IKCurveCubicBezier::GetHandleLocation(const FVector HandleStart, const FVector HandleDir, const float HandleHeight)
{
	const FVector GetHandleLocation = HandleStart + (HandleDir * HandleHeight);
	return GetHandleLocation;
}

/*
 * See https://raphlinus.github.io/curves/2018/12/28/bezier-arclength.html
 */
float IKCurveCubicBezier::GetHandleHeight(const FVector P1, const FVector P2, const float HandleWeight, const float ArcLength)
{
	const FVector P = (P2 - P1);
	const float Lc = P.Size();

	const float A = Lc * HandleWeight;
	const float B = Lc - A;
	const float Lp = 3 * ArcLength - 2 * Lc;
	const float HandleHeight = FMath::Sqrt(FMath::Pow(A, 4) - 2 * FMath::Square(A) * FMath::Square(B) - 2 * FMath::Square(A) * FMath::Square(Lp) + FMath::Pow(B, 4) - 2 * FMath::Square(B) * FMath::Square(Lp) + FMath::Pow(Lp, 4)) / (2 * Lp);

	return HandleHeight;
}

IKCurveCubicBezier* IKCurveCubicBezier::FindCurve(FVector P1, FVector P2, FVector HandleDir, float HandleWeight,
                                                  float TargetArcLength, int MaxIterations, float CurveFitTolerance,
                                                  int NumPoints, TArray<FVector>& ControlPoints, float HandleAngle,
                                                  EIKCurveTypes CurveType)
{
	const FVector P = (P2 - P1);
	const FVector QuadHandleStart = P1 + (P * HandleWeight);
	float HandleHeight = GetHandleHeight(P1, P2, HandleWeight, TargetArcLength);
	float MinHandleHeight = 0;
	float MaxHandleHeight = P.SizeSquared();
	FVector Handle1;
	FVector Handle2;

	FVector const RotationAxis = FVector::CrossProduct(P, HandleDir).GetSafeNormal();
	FVector const RotatedHandleDir1 = HandleDir.RotateAngleAxis(HandleAngle, RotationAxis);
	FVector const RotatedHandleDir2 = HandleDir.RotateAngleAxis(-HandleAngle, RotationAxis);

	IKCurveCubicBezier* Bezier = nullptr;
	for (int i = 0; i < MaxIterations; i++)
	{

		if (CurveType == IK_QuadraticBezier)
		{
			Handle1 = GetHandleLocation(QuadHandleStart, HandleDir, HandleHeight);
			Bezier = new IKCurveCubicBezier(P1, Handle1, P2);
		}
		else // Create a cubic handle
		{
			Handle1 = GetHandleLocation(P1, RotatedHandleDir1, HandleHeight);
			Handle2 = GetHandleLocation(P2, RotatedHandleDir2, HandleHeight);
			Bezier = new IKCurveCubicBezier(P1, Handle1, Handle2, P2);
		}

		Bezier->EvaluateMany(NumPoints);
		float const Delta = Bezier->ArcLength - TargetArcLength;

		if (FMath::Abs(Delta) < CurveFitTolerance) { break; }
		else
		{
			// Height too High
			if (Delta > 0)
			{
				MaxHandleHeight = HandleHeight;
			}
			// Height too Low
			else
			{
				MinHandleHeight = HandleHeight;
			}
			const float Range = MaxHandleHeight - MinHandleHeight;
			HandleHeight = MinHandleHeight + (Range / 2.f);
		}
	}

	ControlPoints[0] = P1;
	ControlPoints[1] = Handle1;
	if(CurveType == IK_CubicBezier) { ControlPoints[2] = Handle2; }
	ControlPoints[ControlPoints.Num() - 1] = P2;

	return Bezier;
}

FVector IKCurveCubicBezier::Evaluate(const float T) const
{
	const auto Pow = FGenericPlatformMath::Pow;

	// Quadratic:
	if (CurveType == IK_QuadraticBezier)
	{
		return Pow(1 - T, 2) * A + (1 - T) * 2 * T * B + (T * T) * C;
	}

	//Cubic
	return (
		A * Pow((1 - T), 3) + 3 * B * Pow((1 - T),2) * T + 3 * C * (1 - T) * Pow(T, 2) + D * Pow(T, 3)
	);

}

FVector IKCurveCubicBezier::EvaluateDerivative(float T) const
{
	const auto Pow = FGenericPlatformMath::Pow;
	// Quadratic:
	if (CurveType == IK_QuadraticBezier)
	{
		return A * (2 * T - 2) + (2 * C - 4 * B) * T + 2 * B;
	}

	//Cubic
	FVector const a = 3 * (B - A);
	FVector const b = 3 * (C - B);
	FVector const c = 3 * (D - C);
	return a * Pow(1 - T, 2) + 2 * b * (1 - T) * T + c * Pow(T, 2);
}

/**
 * See https://stackoverflow.com/questions/25453159/getting-consistent-normals-from-a-3d-cubic-IKCurveCubicbezier-path
 * for a full explanation of this method
 */
FVector IKCurveCubicBezier::EvaluateNormal(float T) const
{
	FVector const R1 = EvaluateDerivative(T);
	FVector const R2 = EvaluateDerivative(T + 0.01);
	float const Q1 = R1.Size();
	float const Q2 = R2.Size();
	FVector const NormalR1 = R1.GetSafeNormal();
	FVector const NormalR2 = R2.GetSafeNormal();
	FVector Cp = FVector::CrossProduct(NormalR2, NormalR1).GetSafeNormal();

	// rotation matrix
	float R[] = {
		Cp.X * Cp.X,
		Cp.X * Cp.Y - Cp.Z,
		Cp.X * Cp.Z + Cp.Y,
		Cp.X * Cp.Y + Cp.Z,
		Cp.Y * Cp.Y,
		Cp.Y * Cp.Z - Cp.X,
		Cp.X * Cp.Z - Cp.Y,
		Cp.Y * Cp.Z + Cp.X,
		Cp.Z * Cp.Z
	};
	// normal vector:
	FVector N;
	N.X = R[0] * R1.X + R[1] * R1.Y + R[2] * R1.Z;
	N.Y = R[3] * R1.X + R[4] * R1.Y + R[5] * R1.Z;
	N.Z = R[6] * R1.X + R[7] * R1.Y + R[8] * R1.Z;

	return N;
}

void IKCurveCubicBezier::EvaluateMany(int32 const NumPoints)
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

FCurvePoint IKCurveCubicBezier::Approximate(float const TargetArcLength)
{
	FCurvePoint NearestCurvePoint = FCurvePoint();
	bool FoundMatch = false;
	int SearchAreaStart = 0;
	int SearchAreaEnd = CurveCache.GetPoints().Num() - 1;
	// The cache is not big enough to search
	if (SearchAreaEnd < 0) { FoundMatch = true; }
	if (SearchAreaStart == SearchAreaEnd)
	{
		NearestCurvePoint.Point = CurveCache.Get(SearchAreaStart).Point;
		NearestCurvePoint.T = 0;
		FoundMatch = true;
	}

	// Check our edges before bothering to search
	const float MinArcLength = CurveCache.Get(SearchAreaStart).ArcLength;
	const float MaxArcLength = CurveCache.Get(SearchAreaEnd).ArcLength;
	if (TargetArcLength <= MinArcLength)
	{
		NearestCurvePoint.Point = CurveCache.Get(SearchAreaStart).Point;
		NearestCurvePoint.T = 0;
		FoundMatch = true;
	}
	if (TargetArcLength >= MaxArcLength)
	{
		NearestCurvePoint.Point = CurveCache.Get(SearchAreaEnd).Point;
		NearestCurvePoint.T = 1;
		FoundMatch = true;
	}

	while (!FoundMatch)
	{
		const float SearchRange = SearchAreaEnd - SearchAreaStart;
		const float Mid = SearchAreaStart + (SearchRange / 2.f);
		const int Right = FMath::CeilToInt(Mid);
		const int Left = Mid - 1;

		const auto LeftCacheItem = CurveCache.Get(Left);
		const auto RightCacheItem = CurveCache.Get(Right);

		// Correct Range
		if (TargetArcLength >= LeftCacheItem.ArcLength && TargetArcLength <= RightCacheItem.ArcLength)
		{
			FoundMatch = true;
			const float Overshoot = TargetArcLength - LeftCacheItem.ArcLength;
			const float ArcLengthGap = RightCacheItem.ArcLength - LeftCacheItem.ArcLength;
			const float PercentThroughGap = Overshoot / ArcLengthGap;

			NearestCurvePoint.Point = FMath::Lerp(LeftCacheItem.Point, RightCacheItem.Point, PercentThroughGap);
			NearestCurvePoint.T = FMath::Lerp(LeftCacheItem.T, RightCacheItem.T, PercentThroughGap);
		}
		else if (TargetArcLength > RightCacheItem.ArcLength) { SearchAreaStart = Right; } // We're too low, go right
		else { SearchAreaEnd = Left; } // We're too high, go left
	}

	NearestCurvePoint.Tangent = EvaluateDerivative(NearestCurvePoint.T).GetSafeNormal();
	NearestCurvePoint.Normal = EvaluateNormal(NearestCurvePoint.T).GetSafeNormal();

	return NearestCurvePoint;
}
