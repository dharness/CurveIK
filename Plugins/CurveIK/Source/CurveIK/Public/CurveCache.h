#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "CurveCache.generated.h"

USTRUCT()
struct CURVEIK_API FCurveIK_CurveCacheItem
{
	GENERATED_BODY()

public:
	float ArcLength;
	float T;
	FVector Point;
	FVector Tangent;
	FVector Normal;
};

USTRUCT()
struct CURVEIK_API FCurveIK_CurveCache
{
	GENERATED_BODY()

public:
	void Add(float ArcLength, FVector CurvePosition, float T);

	FCurveIK_CurveCacheItem Get(int Index);

	void Empty();

	FVector FindNearest(float ArcLength);
	
	TArray<FVector> GetPoints();

private:
	TArray<FCurveIK_CurveCacheItem> CurveCache;
};

