#include "CurveIKCore.h"
#include "CurveCache.h"
#include "IKCurves/IKCurveBezier.h"
#include "Engine/World.h"
#include "IKCurves/IKCurveLine.h"

enum CurveType { Bezier, Line };

namespace CurveIK_AnimationCore
{
	FVector GetDefaultHandleDir(const FVector P1, const FVector P2, const FVector ComponentUpVector,
		FVector ComponentRightVector, FCurveIKDebugData& CurveIKDebugData)
	{
		const FVector P = (P2 - P1).GetSafeNormal();
		//auto PlaneNormal = FVector::CrossProduct(P, ComponentUpVector).GetSafeNormal();
		//auto RightOnPlane = FVector::VectorPlaneProject(ComponentRightVector, PlaneNormal).GetSafeNormal();
		auto RightOnPlane = FVector(P.X, P.Y, 0.f).GetSafeNormal();
		CurveIKDebugData.RightOnPlane = RightOnPlane;
		//
		auto Theta = FMath::Acos(FVector::DotProduct(RightOnPlane, P));
		auto RotationAxis = FVector::CrossProduct(RightOnPlane, P).GetSafeNormal();
		auto DeltaQuat = FQuat(RotationAxis, Theta);
		return DeltaQuat.RotateVector(ComponentUpVector);
		
		//auto Theta = FMath::Acos(FVector::DotProduct(RightOnPlane, P));
		//auto DeltaQuat = FQuat(PlaneNormal, Theta);
		//return DeltaQuat.RotateVector(ComponentUpVector);

		

		//auto v = FVector::CrossProduct(ComponentRightVector, P);
		//float const MatrixOrder = 3;

		//float vx[3 * 3] = {
		//	0, -1*v.Z, v.Y,
		//	v.Z, 0, -1*v.X,
		//	-1*v.Y, v.X, 0
		//};
		//float I[3 * 3] = {
		//	1, 0, 0,
		//	0, 1, 0,
		//	0, 0, 1
		//};
		////auto RowMajorIndex = 3 * i + j;
		//float C = FVector::DotProduct(ComponentRightVector, P);
		//float CP = 1 / (1 + C);
		//TArray<float> R;
		//R.Init(0, 3 * 3);

		//for(int i = 0; i < 9; i++)
		//{
		//	float vx2 = FMath::Square(vx[i]);
		//	R[i] = I[i] + vx[i] + (vx2 * CP);
		//	UE_LOG(LogTemp, Warning, TEXT("%f"), R[i]);
		//}

		//FVector B = ComponentUpVector;
		//FVector result = FVector(
		//	(R[0] * B[0]) + (R[1] * B[1]) + (R[2] * B[2]),
		//	(R[3] * B[0]) + (R[4] * B[1]) + (R[5] * B[2]),
		//	(R[6] * B[0]) + (R[7] * B[1]) + (R[8] * B[2])
		//);
		//return result;
		//return FVector::RightVector;
		
		
		const FQuat Transformation = FQuat::FindBetweenVectors(ComponentUpVector, P);
		UE_LOG(LogTemp, Warning, TEXT("Transformation: %s"), *Transformation.ToString());
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

	IKCurve* FindCurveLine(FVector P1, FVector P2, FVector HandleDir, float TargetArcLength)
	{
		FVector LineDir = P2 - P1;
		return new IKCurveLine(P1, P2, HandleDir, TargetArcLength);
	}

	IKCurve* FindCurveBezier(FVector P1, FVector P2, FVector HandleDir, FVector& HandlePosition, float HandleWeight, float TargetArcLength,
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

		IKCurveBezier* Bezier = nullptr;
		for (int i = 0; i < MaxIterations; i++)
		{
			const FVector Handle = GetHandleLocation(HandleStart, HandleDir, HandleHeight);
			ControlPoints[1] = Handle;
			ControlPoints[2] = Handle;
			HandlePosition = Handle;

			Bezier = new IKCurveBezier(P1, Handle, P2);
			Bezier->EvaluateMany(NumPoints);
			const float Delta = Bezier->ArcLength - TargetArcLength;

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

		const FVector P1 = InOutChain[0].Position;
		FVector P2 = TargetPosition;
		const FVector HandleDir = GetDefaultHandleDir(P1, P2, UpVector, RightVector, FCurveIKDebugData);
		FVector Handle;

		FCurveIKDebugData.RightVector = RightVector;
		FCurveIKDebugData.UpVector = UpVector;
		FCurveIKDebugData.HandleDir = HandleDir;
		FCurveIKDebugData.P1 = P1;
		FCurveIKDebugData.P2 = P2;

		
		float ArcLength = 0;
		const float Weight = FMath::Clamp(ControlPointWeight, 0.0f, 1.0f);
		IKCurve* Curve;
		UE_LOG(LogTemp, Warning, TEXT("MaximumReach: %f"), MaximumReach);
		if (RootToTargetDistSq > FMath::Square(MaximumReach))
		{
			Curve = FindCurveLine(P1, P2, HandleDir, MaximumReach);
			UE_LOG(LogTemp, Warning, TEXT("Line Mode"));
		} else
		{
			Curve = FindCurveBezier(P1, P2, HandleDir, Handle, Weight, MaximumReach, MaxIterations, CurveFitTolerance, NumPointsOnCurve);
		}
		FCurveIKDebugData.ControlPoint = Handle;

		for (int LinkIndex = 0; LinkIndex < NumChainLinks; LinkIndex++)
		{
			FCurveIKChainLink& CurrentLink = InOutChain[LinkIndex];
			ArcLength += CurrentLink.Length;

			const FCurvePoint CurvePoint = Curve->Approximate(ArcLength);
			const FVector BonePosition = CurvePoint.Point;
			CurrentLink.CurvePoint = CurvePoint;
			//UE_LOG(LogTemp, Warning, TEXT("BonePosition: %s"), *BonePosition.ToString());

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
		bBoneLocationUpdated = true;
		return bBoneLocationUpdated;
	}
};
