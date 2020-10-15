#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "BoneIndices.h"
#include "BoneContainer.h"
#include "BonePose.h"
#include "CurveIKCore.h"
#include "IKCurves/IKCurve.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "AnimNode_CurveIK.generated.h"

class FPrimitiveDrawInterface;
class USkeletalMeshComponent;

USTRUCT()
struct FCurveIK_CachedBoneData
{
	GENERATED_BODY()

	FCurveIK_CachedBoneData()
		: Bone(NAME_None)
		, RefSkeletonIndex(INDEX_NONE)
	{
	}

	FCurveIK_CachedBoneData(const FName& InBoneName, int32 InRefSkeletonIndex)
		: Bone(InBoneName)
		, RefSkeletonIndex(InRefSkeletonIndex)
	{
	}

	/** The bone this data applies to */
	UPROPERTY()
	FBoneReference Bone;

	/** Index of the bone in the reference skeleton */
	UPROPERTY()
	int32 RefSkeletonIndex;
};

USTRUCT(BlueprintType)
struct CURVEIK_API FAnimNode_CurveIK : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	/** The location that the IK system attempts to extend towards */
	UPROPERTY(EditAnywhere, Category = Effector, meta = (PinShownByDefault))
	FVector EffectorLocation;

	/** A draggable target used in editing */
	UPROPERTY(EditAnywhere, Category = Effector)
	FBoneSocketTarget EffectorTarget;

	/** Controls the interpretation of FAnimNode_CurveIK::EffectorLocation */
	UPROPERTY(EditAnywhere, Category = Effector)
	TEnumAsByte<enum EBoneControlSpace> EffectorLocationSpace;

	/** Controls how close to the root or tip the control point is placed */
	UPROPERTY(EditAnywhere, Category = Effector, meta = (ClampMin = "0", ClampMax = "1", UIMin = "0", UIMax = "1"))
	float ControlPointWeight;

	/** The rotational offset from the default position of the curves normal. Controls which direction bends in the curve point. */
	UPROPERTY(EditAnywhere, Category = Effector)
	float NormalRotation;

	UPROPERTY(EditAnywhere, Category = Solver)
	TEnumAsByte<enum EIKCurveTypes> CurveType;

	/** Name of tip bone */
	UPROPERTY(EditAnywhere, Category = Solver)
	FBoneReference TipBone;

	/** Name of the root bone*/
	UPROPERTY(EditAnywhere, Category = Solver)
	FBoneReference RootBone;

	/** Maximum number of iterations allowed, to control performance. */
	UPROPERTY(EditAnywhere, Category = Solver)
	int32 MaxIterations;

	/** The number of points used to approximate the curve. Higher values impact both compute times and memory. */
	UPROPERTY(EditAnywhere, Category = Solver)
	int32 CurveDetail;

	/** Allowable delta between arc length of the curve and arc length as the sum of bone lengths */
	UPROPERTY(EditAnywhere, Category = Solver)
	float CurveFitTolerance;

	/** Degree to which the bones may stretch to approximate the curve more accurately. Non-zero values may cause the tip to fail to align with target. */
	UPROPERTY(EditAnywhere, Category = Solver, meta = (ClampMin = "0", ClampMax = "1", UIMin = "0", UIMax = "1"))
	float Stretch;

	UPROPERTY(EditAnywhere, Category = Solver, meta = (ClampMin = "0", ClampMax = "360", UIMin = "0", UIMax = "360"))
	float HandleAngle;

#if WITH_EDITORONLY_DATA
	UPROPERTY(EditAnywhere, Category = Debug)
	/** Toggle drawing of axes to debug joint rotation*/
	bool bEnableDebugDraw;

	UPROPERTY(EditAnywhere, Category = Debug)
	bool bShowNormals;

	UPROPERTY(EditAnywhere, Category = Debug)
	bool bShowTangents;

	UPROPERTY(EditAnywhere, Category = Debug)
	bool bShowBoneDirection;

	UPROPERTY(EditAnywhere, Category = Debug)
	bool bShowLinks;
#endif

public:
	FAnimNode_CurveIK();

	// FAnimNode_Base interface
	virtual void GatherDebugData(FNodeDebugData& DebugData) override;
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	// End of FAnimNode_Base interface

	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

	virtual void ConditionalDebugDraw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* PreviewSkelMeshComp) const;

private:
	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	void GatherBoneReferences(const FReferenceSkeleton& RefSkeleton);
	void GatherBoneLengths(const FReferenceSkeleton& RefSkeleton);
	// End of FAnimNode_SkeletalControlBase interface

	// Convenience function to get current (pre-translation iteration) component space location of bone by bone index
	FVector GetCurrentLocation(FCSPose<FCompactPose>& MeshBases, const FCompactPoseBoneIndex& BoneIndex);
	static FTransform GetTargetTransform(const FTransform& InComponentTransform, FCSPose<FCompactPose>& MeshBases, FBoneSocketTarget& InTarget, EBoneControlSpace Space, const FTransform& InOffset);

	/** Cached data for bones in the IK chain, from start to end */
	TArray<FCurveIK_CachedBoneData> CachedBoneReferences;

	/** Cached bone lengths. Same size as CachedBoneReferences */
	TArray<float> CachedBoneLengths;


#if WITH_EDITOR
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
public:
	FCurveIKDebugData CurveIKDebugData;
	TArray<FCurveIKChainLink> Chain;
#endif
#endif
};
