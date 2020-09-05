#pragma once

#include "CoreMinimal.h"
#include "CurveCache.h"

#include "Bezier.generated.h"

USTRUCT()
struct FBezier
{
	GENERATED_BODY()

	public:
		UPROPERTY()
		FVector A;
	
		UPROPERTY()
		FVector B;
	
		UPROPERTY()
		FVector C;

		UPROPERTY()
		FCurveIK_CurveCache CurveCache;

		UPROPERTY()
		float ArcLength;

		FBezier()
			: A(FVector::ZeroVector)
			, B(FVector::ZeroVector)
			, C(FVector::ZeroVector)
		{}

		FBezier(FVector const A, FVector const B, FVector const C): A(A), B(B), C(C) {}

		FVector Evaluate(float T) const;
	
		FVector EvaluateDerivative(float T) const;
	FVector EvaluateNormal(float T) const;

	void EvaluateMany(int32 NumPoints);

		FCurvePoint Approximate(float TargetArcLength);
};

