#include "CurveCache.h"

void FCurveIK_CurveCache::Add(float ArcLength, FVector CurvePosition, float T)
{
	auto Item = FCurveIK_CurveCacheItem();
	Item.ArcLength = ArcLength;
	Item.Point = CurvePosition;
	Item.T = T;
	CurveCache.Add(Item);
}

FCurveIK_CurveCacheItem FCurveIK_CurveCache::Get(int Index)
{
	return CurveCache[Index];
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
	if (SearchAreaStart == SearchAreaEnd) { return CurveCache[SearchAreaStart].Point; }

	// Check our edges before bothering to search
	const float MinArcLength = CurveCache[SearchAreaStart].ArcLength;
	const float MaxArcLength = CurveCache[SearchAreaEnd].ArcLength;
	if (ArcLength <= MinArcLength) { return CurveCache[SearchAreaStart].Point; }
	if (ArcLength >= MaxArcLength) { return CurveCache[SearchAreaEnd].Point; }

	while (!FoundMatch)
	{
		const float SearchRange = SearchAreaEnd - SearchAreaStart;
		const float Mid = SearchAreaStart + (SearchRange / 2.f);
		const int Right = FMath::CeilToInt(Mid);
		const int Left = Mid - 1;

		const auto LeftCacheItem = CurveCache[Left];
		const auto RightCacheItem = CurveCache[Right];

		// Correct Range
		if (ArcLength >= LeftCacheItem.ArcLength && ArcLength <= RightCacheItem.ArcLength)
		{
			FoundMatch = true;
			const float Overshoot = ArcLength - LeftCacheItem.ArcLength;
			const float ArcLengthGap = RightCacheItem.ArcLength - LeftCacheItem.ArcLength;
			const float PercentThroughGap = Overshoot / ArcLengthGap;
			NearestCurveValue = FMath::Lerp(LeftCacheItem.Point, RightCacheItem.Point, PercentThroughGap);
		}
		else if (ArcLength > RightCacheItem.ArcLength) { SearchAreaStart = Right; } // We're too low, go right
		else { SearchAreaEnd = Left; } // We're too high, go left
	}

	return NearestCurveValue;
}

TArray<FVector> FCurveIK_CurveCache::GetPoints()
{
	TArray<FVector> Points;
	for (auto CacheItem : CurveCache)
	{
		Points.Add(CacheItem.Point);
	}
	return  Points;
}
