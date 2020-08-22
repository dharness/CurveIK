#include "AnimGraphNode_CurveIK.h"
#include "Animation/AnimInstance.h"
#include "AnimNodeEditModes.h"
#include "AnimationCustomVersion.h"
#include "CurveIKEditModes.h"


const FEditorModeID CurveIKEditModes::CurveIK("UAnimGraphNode_CurveIK.CurveIK");

UAnimGraphNode_CurveIK::UAnimGraphNode_CurveIK(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_CurveIK::GetControllerDescription() const
{
	return FText::FromString(FString("Curve IK"));
}

void UAnimGraphNode_CurveIK::Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* PreviewSkelMeshComp) const
{
	if (PreviewSkelMeshComp)
	{
		if (FAnimNode_CurveIK* ActiveNode = GetActiveInstanceNode<FAnimNode_CurveIK>(PreviewSkelMeshComp->GetAnimInstance()))
		{
			ActiveNode->ConditionalDebugDraw(PDI, PreviewSkelMeshComp);
		}
	}
}

FText UAnimGraphNode_CurveIK::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

void UAnimGraphNode_CurveIK::CopyNodeDataToPreviewNode(FAnimNode_Base* InPreviewNode)
{
	FAnimNode_CurveIK* AnimNodeCurveIK = static_cast<FAnimNode_CurveIK*>(InPreviewNode);

	// copies Pin values from the internal node to get data which are not compiled yet
	AnimNodeCurveIK->EffectorLocation = Node.EffectorLocation;
}

FEditorModeID UAnimGraphNode_CurveIK::GetEditorMode() const
{
	return CurveIKEditModes::CurveIK;
}

void UAnimGraphNode_CurveIK::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);

	Ar.UsingCustomVersion(FAnimationCustomVersion::GUID);
}
