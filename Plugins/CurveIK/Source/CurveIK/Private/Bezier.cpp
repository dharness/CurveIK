#include "Bezier.h"

FVector FBezier::Evaluate(const float T) const
{
	const auto Pow = FGenericPlatformMath::Pow;

	// Quadratic:
	return Pow(1 - T, 2) * A + (1 - T) * 2 * T * B + (T * T) * C;
}

FVector FBezier::EvaluateDerivative(float T) const
{
	const auto Pow = FGenericPlatformMath::Pow;
	// Quadratic:
	FVector const Derivative = A * (2*T - 2) + (2 * C - 4 * B) * T + 2 * B;
	return Derivative;
}

FVector FBezier::EvaluateNormal(float T) const
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
		Cp.X* Cp.X,
		Cp.X* Cp.Y - Cp.Z,
		Cp.X* Cp.Z + Cp.Y,
		Cp.X* Cp.Y + Cp.Z,
		Cp.Y* Cp.Y,
		Cp.Y* Cp.Z - Cp.X,
		Cp.X* Cp.Z - Cp.Y,
		Cp.Y* Cp.Z + Cp.X,
		Cp.Z* Cp.Z
	};
	// normal vector:
	FVector N;
	N.X = R[0] * R1.X + R[1] * R1.Y + R[2] * R1.Z;
	N.Y = R[3] * R1.X + R[4] * R1.Y + R[5] * R1.Z;
	N.Z = R[6] * R1.X + R[7] * R1.Y + R[8] * R1.Z;

	return N;
}

void FBezier::EvaluateMany(int32 const NumPoints)
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

FCurveIK_CurveCacheItem FBezier::Approximate(float const TargetArcLength)
{
	FCurveIK_CurveCacheItem NearestCurvePoint = FCurveIK_CurveCacheItem();
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
