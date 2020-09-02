#include "Bezier.h"

FVector FBezier::Evaluate(const float T) const
{
	const auto Pow = FGenericPlatformMath::Pow;

	FVector PFinal;
	PFinal.X = Pow(1 - T, 2) * A.X +
		(1 - T) * 2 * T * B.X +
		T * T * D.X;
	PFinal.Y = Pow(1 - T, 2) * A.Y +
		(1 - T) * 2 * T * B.Y +
		T * T * D.Y;
	PFinal.Z = Pow(1 - T, 2) * A.Z +
		(1 - T) * 2 * T * B.Z +
		T * T * D.Z;
	return PFinal;
}

FVector FBezier::EvaluateDerivative(float T) const
{
	const auto Pow = FGenericPlatformMath::Pow;

	const FVector APrime = 3 * (B - A);
	const FVector BPrime = 3 * (C - B);
	const FVector CPrime = 3 * (D - C);
	return APrime * Pow(1 - T, 2) + 2 * BPrime * (1 - T) * T + CPrime * Pow(T, 2);
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

FVector FBezier::Approximate(float const TargetArcLength)
{
	FVector NearestCurveValue;
	bool FoundMatch = false;
	int SearchAreaStart = 0;
	int SearchAreaEnd = CurveCache.GetPoints().Num() - 1;
	// The cache is not big enough to search
	if (SearchAreaEnd < 0) { return FVector::ZeroVector; }
	if (SearchAreaStart == SearchAreaEnd) { return CurveCache.Get(SearchAreaStart).Point; }

	// Check our edges before bothering to search
	const float MinArcLength = CurveCache.Get(SearchAreaStart).ArcLength;
	const float MaxArcLength = CurveCache.Get(SearchAreaEnd).ArcLength;
	if (TargetArcLength <= MinArcLength) { return CurveCache.Get(SearchAreaStart).Point; }
	if (TargetArcLength >= MaxArcLength) { return CurveCache.Get(SearchAreaEnd).Point; }

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
			NearestCurveValue = FMath::Lerp(LeftCacheItem.Point, RightCacheItem.Point, PercentThroughGap);
		}
		else if (TargetArcLength > RightCacheItem.ArcLength) { SearchAreaStart = Right; } // We're too low, go right
		else { SearchAreaEnd = Left; } // We're too high, go left
	}

	return NearestCurveValue;
}
