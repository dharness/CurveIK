#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "CurveCache.generated.h"

USTRUCT()
struct CURVEIK_API FCurveIK_CurveCache
{
	GENERATED_BODY()

public:
	void Add(float ArcLength, FVector CurvePosition);

	void Empty();

	FVector FindNearest(float ArcLength);
	
	TArray<FVector> GetPoints();

private:
	TArray<TTuple<float, FVector>> CurveCache;
};