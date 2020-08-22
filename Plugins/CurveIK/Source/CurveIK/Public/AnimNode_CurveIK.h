#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "BoneIndices.h"
#include "BoneContainer.h"
#include "BonePose.h"
#include "CurveIKCore.h"
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
	{}

	FCurveIK_CachedBoneData(const FName& InBoneName, int32 InRefSkeletonIndex)
		: Bone(InBoneName)
		, RefSkeletonIndex(InRefSkeletonIndex)
	{}

	/** The bone we refer to */
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

		UPROPERTY(EditAnywhere, Category = Effector, meta = (PinShownByDefault))
		FVector EffectorLocation;

	UPROPERTY(EditAnywhere, Category = Poles, meta = (PinShownByDefault))
		FTransform Bead;

	UPROPERTY(EditAnywhere, Category = Effector)
		FBoneSocketTarget EffectorTarget;

	UPROPERTY(EditAnywhere, Category = Effector)
		TEnumAsByte<enum EBoneControlSpace> EffectorLocationSpace;

	UPROPERTY(EditAnywhere, Category = Effector, meta = (ClampMin = "0", ClampMax = "1", UIMin = "0", UIMax = "1"))
		float ControlPointWeight;

	/** Name of tip bone */
	UPROPERTY(EditAnywhere, Category = Solver)
		FBoneReference TipBone;

	/** Name of the root bone*/
	UPROPERTY(EditAnywhere, Category = Solver)
		FBoneReference RootBone;


	UPROPERTY(EditAnywhere, Category = Poles, meta = (ClampMin = "0", ClampMax = "360", UIMin = "0", UIMax = "360"))
		float PoleAngle;

#if WITH_EDITORONLY_DATA
	/** Toggle drawing of axes to debug joint rotation*/
	UPROPERTY(EditAnywhere, Category = Solver)
		bool bEnableDebugDraw;
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
