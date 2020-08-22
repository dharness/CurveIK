#include "CurveCache.h"

void FCurveIK_CurveCache::Add(float ArcLength, FVector CurvePosition)
{
	CurveCache.Add(MakeTuple(ArcLength, CurvePosition));
}

void FCurveIK_CurveCache::Empty()
{
	CurveCache.Empty();
}

FVector FCurveIK_CurveCache::FindNearest(const float ArcLength)
{
	FVector NearestCurveValue;
	bool FoundMatch = false;
	int SearchAreaStart = 0;
	int SearchAreaEnd = CurveCache.Num() - 1;
	// The cache is not big enough to search
	if (SearchAreaEnd < 0) { return FVector::ZeroVector; }
	if (SearchAreaStart == SearchAreaEnd) { return CurveCache[SearchAreaStart].Value; }

	// Check our edges before bothering to search
	const float MinArcLength = CurveCache[SearchAreaStart].Key;
	const float MaxArcLength = CurveCache[SearchAreaEnd].Key;
	if (ArcLength <= MinArcLength) { return CurveCache[SearchAreaStart].Value; }
	if (ArcLength >= MaxArcLength) { return CurveCache[SearchAreaEnd].Value; }

	while (!FoundMatch)
	{
		const float SearchRange = SearchAreaEnd - SearchAreaStart;
		const float Mid = SearchAreaStart + (SearchRange / 2.f);
		const int Right = FMath::CeilToInt(Mid);
		const int Left = Mid - 1;

		const auto LeftCacheItem = CurveCache[Left];
		const auto RightCacheItem = CurveCache[Right];

		// Correct Range
		if (ArcLength >= LeftCacheItem.Key && ArcLength <= RightCacheItem.Key)
		{
			FoundMatch = true;
			const float Overshoot = ArcLength - LeftCacheItem.Key;
			const float ArcLengthGap = RightCacheItem.Key - LeftCacheItem.Key;
			const float PercentThroughGap = Overshoot / ArcLengthGap;
			NearestCurveValue = FMath::Lerp(LeftCacheItem.Value, RightCacheItem.Value, PercentThroughGap);
		}
		else if (ArcLength > RightCacheItem.Key) { SearchAreaStart = Right; } // We're too low, go right
		else { SearchAreaEnd = Left; } // We're too high, go left
	}

	return NearestCurveValue;
}

TArray<FVector> FCurveIK_CurveCache::GetPoints()
{
	TArray<FVector> Points;
	for (auto CacheItem : CurveCache)
	{
		Points.Add(CacheItem.Value);
	}
	return  Points;
}
