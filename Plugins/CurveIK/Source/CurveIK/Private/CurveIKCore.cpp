#include "CurveIKCore.h"
#include "CurveCache.h"
#include "Bezier.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"

namespace CurveIK_AnimationCore
{
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

	FBezier FindCurve(FVector P1, FVector P2, FVector HandleDir, FVector& HandlePosition, float HandleWeight, float TargetArcLength,
	                              int MaxIterations, float CurveFitTolerance, int NumPoints)
	{
		const FVector P = (P2 - P1);
		const FVector HandleStart = P1 + (P * HandleWeight);
		float HandleHeight = GetHandleHeight(P1, P2, HandleWeight, TargetArcLength);
		float MinHandleHeight = 0;
		float MaxHandleHeight = P.SizeSquared();

		FVector ControlPoints[4];
		ControlPoints[0] = P1;
		ControlPoints[3] = P2;

		FBezier Bezier;
		for (int i = 0; i < MaxIterations; i++)
		{
			const FVector Handle = GetHandleLocation(HandleStart, HandleDir, HandleHeight);
			ControlPoints[1] = Handle;
			ControlPoints[2] = Handle;
			HandlePosition = Handle;

			Bezier = FBezier(P1, Handle, P2);
			Bezier.EvaluateMany(NumPoints);
			const float Delta = Bezier.ArcLength - TargetArcLength;

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

		return Bezier;
	}


	bool SolveCurveIK(TArray<FCurveIKChainLink>& InOutChain, const FVector& TargetPosition, float ControlPointWeight,
	                  float MaximumReach, int MaxIterations, float CurveFitTolerance, int NumPointsOnCurve, float Stretch,
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
			FVector Handle;

			FCurveIKDebugData.RightVector = RightVector;
			FCurveIKDebugData.UpVector = UpVector;
			FCurveIKDebugData.ControlVector = HandleDir;
			FCurveIKDebugData.P1 = P1;
			FCurveIKDebugData.P2 = P2;

			float ArcLength = 0;
			const float Weight = FMath::Clamp(ControlPointWeight, 0.0f, 1.0f);
			FBezier Bezier = FindCurve(P1, P2, HandleDir, Handle, Weight, MaximumReach, MaxIterations, CurveFitTolerance, NumPointsOnCurve);
			//FCurveIKDebugData.CurveCache = CurveCache;
			FCurveIKDebugData.ControlPoint = Handle;

			for (int LinkIndex = 0; LinkIndex < NumChainLinks; LinkIndex++)
			{
				FCurveIKChainLink& CurrentLink = InOutChain[LinkIndex];
				ArcLength += CurrentLink.Length;

				const FCurveIK_CurveCacheItem CurvePoint = Bezier.Approximate(ArcLength);
				const FVector BonePosition = CurvePoint.Point;
				CurrentLink.Tangent = CurvePoint.Tangent;
				CurrentLink.Normal = CurvePoint.Normal;

				if (Stretch != 0)
				{
					const float T = ArcLength / MaximumReach;
					const FVector StretchedBonePosition = Bezier.Evaluate(T);
					CurrentLink.Position = FMath::Lerp(BonePosition, StretchedBonePosition, Stretch);
				} else
				{
					CurrentLink.Position = BonePosition;
				}
			}
			bBoneLocationUpdated = true;
		}
		return bBoneLocationUpdated;
	}
};
