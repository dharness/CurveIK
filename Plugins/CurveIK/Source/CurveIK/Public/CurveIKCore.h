#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "BoneIndices.h"
#include "CurveCache.h"

#include "CurveIKCore.generated.h"


USTRUCT()
struct FCurveIKChainLink
{
	GENERATED_BODY()

public:
	/** Position of bone in component space. */
	FVector Position;

	/** Distance to its parent link. */
	float Length;

	/* The point on the curve where the base of this link lies */
	FCurvePoint CurvePoint;

	/* The true down direction of the bone's transform after being placed and rotated by the solver */
	FVector BoneDownVector;

	/** Bone Index in SkeletalMesh */
	int32 BoneIndex;

	/** Transform Index that this control will output */
	int32 TransformIndex;

	/** Default Direction to Parent */
	FVector DefaultDirToParent;

	/** Child bones which are overlapping this bone.
	* They have a zero length distance, so they will inherit this bone's transformation. */
	TArray<int32> ChildZeroLengthTransformIndices;

	FCurveIKChainLink()
		: Position(FVector::ZeroVector)
		, Length(0.f)
		, BoneIndex(INDEX_NONE)
		, TransformIndex(INDEX_NONE)
		, DefaultDirToParent(FVector(-1.f, 0.f, 0.f))
	{
	}

	FCurveIKChainLink(const FVector& InPosition, const float InLength, const FCompactPoseBoneIndex& InBoneIndex, const int32& InTransformIndex)
		: Position(InPosition)
		, Length(InLength)
		, BoneIndex(InBoneIndex.GetInt())
		, TransformIndex(InTransformIndex)
		, DefaultDirToParent(FVector(-1.f, 0.f, 0.f))
	{
	}

	FCurveIKChainLink(const FVector& InPosition, const float InLength, const FCompactPoseBoneIndex& InBoneIndex, const int32& InTransformIndex, const FVector& InDefaultDirToParent)
		: Position(InPosition)
		, Length(InLength)
		, BoneIndex(InBoneIndex.GetInt())
		, TransformIndex(InTransformIndex)
		, DefaultDirToParent(InDefaultDirToParent)
	{
	}

	FCurveIKChainLink(const FVector& InPosition, const float InLength, const int32 InBoneIndex, const int32 InTransformIndex)
		: Position(InPosition)
		, Length(InLength)
		, BoneIndex(InBoneIndex)
		, TransformIndex(InTransformIndex)
		, DefaultDirToParent(FVector(-1.f, 0.f, 0.f))
	{
	}
};

USTRUCT()
struct FCurveIKDebugData
{
	GENERATED_BODY()

public:
	FVector ControlPoint;
	FVector HandleDir;
	FVector P1;
	FVector P2;
	FVector RightVector;
	FVector UpVector;
	FVector RightOnPlane;

	FCurveIK_CurveCache CurveCache;

};

namespace CurveIK_AnimationCore
{
	CURVEIK_API bool SolveCurveIK(TArray<FCurveIKChainLink>& InOutChain, const FVector& TargetLocation,
	                              float ControlPointWeight, float MaximumReach, int MaxIterations, float CurveFitTolerance,
	                              int NumPointsOnCurve, float Stretch, FCurveIKDebugData& CurveIKDebugData);
};
