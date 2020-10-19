#include "CurveIKCore.h"
#include "CurveCache.h"
#include "IKCurves/IKCurveBezier.h"
#include "Engine/World.h"
#include "IKCurves/IKCurveCubicBezier.h"
#include "IKCurves/IKCurveLine.h"

enum CurveType { Bezier, Line };

namespace CurveIK_AnimationCore
{
	
	/**
	 * Computes a stable direction vector normal to the direction of ik target and
	 * relative to the component's transform
	 * 
	 * @param P1 The position of the root bone
	 * @param P2 The position of the tip bone
	 * @param ComponentUpVector The up vector of the component to which this IK system applies
	 * @param CurveIKDebugData
	 * 
	 * @return A stable vector normal to P1 and P2
	 */
	FVector GetReferenceNormal(const FVector P1, const FVector P2, const FVector ComponentUpVector,
	                            FCurveIKDebugData& CurveIKDebugData)
	{
		const FVector P_ = (P2 - P1).GetSafeNormal();
		const FVector V = FVector::DownVector - P_;
		
		return -1 * FVector::VectorPlaneProject(V, P_).GetSafeNormal();
	}

	// Implementation of the curve IK algorithm
	bool SolveCurveIK(TArray<FCurveIKChainLink>& InOutChain, const FVector& TargetPosition, float ControlPointWeight,
	                  float MaximumReach, int MaxIterations, float CurveFitTolerance, int NumPointsOnCurve, float Stretch,
	                  FCurveIKDebugData& FCurveIKDebugData, float HandleAngle, EIKCurveTypes CurveType)
	{
		float const RootToTargetDistSq = FVector::DistSquared(InOutChain[0].Position, TargetPosition);
		int32 const NumChainLinks = InOutChain.Num();
		FVector const RightVector = FVector::RightVector;
		FVector const UpVector = FVector::UpVector;

		FVector const P1 = InOutChain[0].Position;
		FVector const P2 = TargetPosition;
		FVector const HandleDir = GetReferenceNormal(P1, P2, UpVector, FCurveIKDebugData);
		TArray<FVector> ControlPoints;
		
		float ArcLength = 0;
		const float Weight = FMath::Clamp(ControlPointWeight, 0.0f, 1.0f);
		IKCurve* Curve;

		if (RootToTargetDistSq > FMath::Square(MaximumReach))
		{
			Curve = IKCurveLine::FindCurve(P1, P2, HandleDir, MaximumReach);
		}
		else
		{
			if (CurveType == IK_QuadraticBezier) { ControlPoints.SetNum(3); }
			else { ControlPoints.SetNum(4); }
			Curve = IKCurveCubicBezier::FindCurve(P1, P2, HandleDir, Weight, MaximumReach, MaxIterations,
			                                      CurveFitTolerance, NumPointsOnCurve, ControlPoints, HandleAngle, CurveType);
		}

		for (int LinkIndex = 0; LinkIndex < NumChainLinks; LinkIndex++)
		{
			FCurveIKChainLink& CurrentLink = InOutChain[LinkIndex];
			ArcLength += CurrentLink.Length;

			const FCurvePoint CurvePoint = Curve->Approximate(ArcLength);
			const FVector BonePosition = CurvePoint.Point;
			CurrentLink.CurvePoint = CurvePoint;

			if (Stretch != 0)
			{
				const float T = ArcLength / MaximumReach;
				const FVector StretchedBonePosition = Curve->Evaluate(T);
				CurrentLink.Position = FMath::Lerp(BonePosition, StretchedBonePosition, Stretch);
			} else
			{
				CurrentLink.Position = BonePosition;
			}
		}

		#if WITH_EDITOR
				FCurveIKDebugData.ControlPoints = ControlPoints;
				FCurveIKDebugData.RightVector = RightVector;
				FCurveIKDebugData.UpVector = UpVector;
				FCurveIKDebugData.HandleDir = HandleDir;
				FCurveIKDebugData.P1 = P1;
				FCurveIKDebugData.P2 = P2;
		#endif // WITH_EDITOR

		return true;
	}
};
