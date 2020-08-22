#include "CurveIKCore.h"
#include "CurveCache.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"

namespace CurveIK_AnimationCore
{
	/*
	 * Calculates a point on a quadratic bezier parameterized by T given a set of points defining the control polygon
	 * @param P1
	 */
	FVector QuadraticBezier(const FVector P1, const FVector Control, const FVector P2, const float T)
	{
		FVector PFinal;
		PFinal.X = FGenericPlatformMath::Pow(1 - T, 2) * P1.X +
			(1 - T) * 2 * T * Control.X +
			T * T * P2.X;
		PFinal.Y = FGenericPlatformMath::Pow(1 - T, 2) * P1.Y +
			(1 - T) * 2 * T * Control.Y +
			T * T * P2.Y;
		PFinal.Z = FGenericPlatformMath::Pow(1 - T, 2) * P1.Z +
			(1 - T) * 2 * T * Control.Z +
			T * T * P2.Z;
		return PFinal;
	}

	FVector GetDefaultHandleDir(const FVector P1, const FVector P2, const FVector ComponentUpVector,
		FVector ComponentRightVector)
	{
		const FVector P = (P2 - P1);
		const FVector DirToNextPoint = P.GetSafeNormal();
		const FQuat Transformation = FQuat::FindBetweenVectors(ComponentUpVector, P);
		const FVector HandleVector = Transformation.RotateVector(ComponentRightVector);

		return HandleVector.GetSafeNormal();
	}

	FVector GetHandleLocation(const FVector HandleStart, const FVector HandleDir, const float HandleHeight)
	{
		const FVector GetHandleLocation = HandleStart + (HandleDir * HandleHeight);
		return GetHandleLocation;
	}
	/*
	 * See https://raphlinus.github.io/curves/2018/12/28/bezier-arclength.html
	 */
	float GetHandleHeight(const FVector P1, const FVector P2, const float HandleWeight, const float ArcLength)
	{
		const FVector P = (P2 - P1);
		const float Lc = P.Size();

		const float A = Lc * HandleWeight;
		const float B = Lc - A;
		const float Lp = 3 * ArcLength - 2 * Lc;
		const float HandleHeight = FMath::Sqrt(FMath::Pow(A, 4) - 2 * FMath::Square(A) * FMath::Square(B) - 2 * FMath::Square(A) * FMath::Square(Lp) + FMath::Pow(B, 4) - 2 * FMath::Square(B) * FMath::Square(Lp) + FMath::Pow(Lp, 4)) / (2 * Lp);

		return HandleHeight;
	}

	float EvaluateBezierCurve(const FVector* ControlPoints, int32 NumPoints, FCurveIK_CurveCache& CurveCache)
	{
		// Because we only use 1 handle instead of 2, B == C
		const FVector& A = ControlPoints[0];
		const FVector& B = ControlPoints[1];
		const FVector& C = ControlPoints[2];
		const FVector& D = ControlPoints[3];

		const float MinT = 0;
		const float MaxT = 1;
		const float StepSize = MaxT / (NumPoints - 1);
		float T = MinT;
		float ArcLength = 0;

		CurveCache.Empty();

		FVector PrevPoint = QuadraticBezier(A, B, D, T);
		CurveCache.Add(ArcLength, PrevPoint);

		for (int i = 1; i < NumPoints; i++)
		{
			T += StepSize;
			const FVector Point = QuadraticBezier(A, B, D, T);
			ArcLength += FVector::Dist(PrevPoint, Point);
			PrevPoint = Point;

			CurveCache.Add(ArcLength, Point);
		}

		return ArcLength;
	}

	FCurveIK_CurveCache FindCurve(FVector P1, FVector P2, FVector HandleDir, float HandleWeight, float TargetArcLength, int MaxIterations, float CurveFitTolerance, int NumPoints, FCurveIKDebugData& FCurveIKDebugData)
	{
		FCurveIK_CurveCache CurveCache = FCurveIK_CurveCache();
		const FVector P = (P2 - P1);
		const FVector HandleStart = P1 + (P * HandleWeight);
		float HandleHeight = GetHandleHeight(P1, P2, HandleWeight, TargetArcLength);
		float MinHandleHeight = 0;
		float MaxHandleHeight = P.SizeSquared();

		FVector ControlPoints[4];
		ControlPoints[0] = P1;
		ControlPoints[3] = P2;

		UE_LOG(LogTemp, Warning, TEXT("----------------------"));
		for (int i = 0; i < MaxIterations; i++)
		{
			const FVector Handle = GetHandleLocation(HandleStart, HandleDir, HandleHeight);
			ControlPoints[1] = Handle;
			ControlPoints[2] = Handle;
			FCurveIKDebugData.ControlPoint = Handle;

			const float ArcLength = EvaluateBezierCurve(ControlPoints, NumPoints, CurveCache);
			const float Delta = ArcLength - TargetArcLength;

			if (FMath::Abs(Delta) < CurveFitTolerance)
			{
				UE_LOG(LogTemp, Warning, TEXT("Delta: %f"), Delta);
				break;
			}
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

		return CurveCache;
	}


	bool SolveCurveIK(TArray<FCurveIKChainLink>& InOutChain, const FVector& TargetPosition, float ControlPointWeight,
	                  float MaximumReach, int MaxIterations, float CurveFitTolerance, int NumPointsOnCurve,
	                  FCurveIKDebugData& FCurveIKDebugData)
	{
		bool bBoneLocationUpdated = false;
		float const RootToTargetDistSq = FVector::DistSquared(InOutChain[0].Position, TargetPosition);
		int32 const NumChainLinks = InOutChain.Num();
		const FVector RightVector = FVector::RightVector;
		const FVector UpVector = FVector::UpVector;

		// If the effector is further away than the distance from root to tip, simply move all bones in a line from root to effector location
		if (RootToTargetDistSq > FMath::Square(MaximumReach))
		{
			for (int32 LinkIndex = 1; LinkIndex < NumChainLinks; LinkIndex++)
			{
				FCurveIKChainLink const& ParentLink = InOutChain[LinkIndex - 1];
				FCurveIKChainLink& CurrentLink = InOutChain[LinkIndex];
				CurrentLink.Position = ParentLink.Position + (TargetPosition - ParentLink.Position).GetUnsafeNormal() * CurrentLink.Length;
			}
			bBoneLocationUpdated = true;
		}
		else // Effector is within reach, calculate bone translations to position tip at effector location
		{
			const FVector P1 = InOutChain[0].Position;
			const FVector P2 = TargetPosition;
			const FVector HandleDir = GetDefaultHandleDir(P1, P2, RightVector, UpVector);

			FCurveIKDebugData.RightVector = RightVector;
			FCurveIKDebugData.UpVector = UpVector;
			FCurveIKDebugData.ControlVector = HandleDir;
			FCurveIKDebugData.P1 = P1;
			FCurveIKDebugData.P2 = P2;

			float ArcLength = 0;
			const float Weight = FMath::Clamp(ControlPointWeight, 0.0f, 1.0f);
			FCurveIK_CurveCache CurveCache = FindCurve(P1, P2, HandleDir, Weight, MaximumReach, MaxIterations, CurveFitTolerance, NumPointsOnCurve, FCurveIKDebugData);
			FCurveIKDebugData.CurveCache = CurveCache;

			for (int LinkIndex = 0; LinkIndex < NumChainLinks; LinkIndex++)
			{
				FCurveIKChainLink& CurrentLink = InOutChain[LinkIndex];
				ArcLength += CurrentLink.Length;
				const FVector BonePosition = CurveCache.FindNearest(ArcLength);
				CurrentLink.Position = BonePosition;
			}
			bBoneLocationUpdated = true;
		}
		return bBoneLocationUpdated;
	}
};
