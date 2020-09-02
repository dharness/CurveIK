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
		FVector D;

		UPROPERTY()
		FCurveIK_CurveCache CurveCache;

		UPROPERTY()
		float ArcLength;

		FBezier()
			: A(FVector::ZeroVector)
			, B(FVector::ZeroVector)
			, C(FVector::ZeroVector)
			, D(FVector::ZeroVector)
		{}

		FBezier(FVector const A, FVector const B, FVector const C, FVector const D): A(A), B(B), C(C), D(D) {}

		FBezier(FVector const A, FVector const B, FVector const D): A(A), B(B), C(B), D(D) {}

		FVector Evaluate(float T) const;
	
		FVector EvaluateDerivative(float T) const;

		void EvaluateMany(int32 NumPoints);

		FVector Approximate(float ArcLength);
};

