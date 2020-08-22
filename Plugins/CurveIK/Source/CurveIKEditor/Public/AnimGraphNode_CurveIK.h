#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "AnimGraphNode_SkeletalControlBase.h"
#include "AnimNode_CurveIK.h"
#include "AnimGraphNode_CurveIK.generated.h"


class FPrimitiveDrawInterface;
class USkeletalMeshComponent;

UCLASS()
class UAnimGraphNode_CurveIK : public UAnimGraphNode_SkeletalControlBase
{
	GENERATED_UCLASS_BODY()

		UPROPERTY(EditAnywhere, Category = Settings)
		FAnimNode_CurveIK Node;

public:
	// UObject interface
	virtual void Serialize(FArchive& Ar) override;
	// End of UObject interface

	// UEdGraphNode interface
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	// End of UEdGraphNode interface

	// UAnimGraphNode_Base interface
	virtual void CopyNodeDataToPreviewNode(FAnimNode_Base* AnimNode) override;
	virtual FEditorModeID GetEditorMode() const override;
	virtual void Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* PreviewSkelMeshComp) const override;
	// End of UAnimGraphNode_Base interface

protected:
	// UAnimGraphNode_SkeletalControlBase interface
	virtual FText GetControllerDescription() const override;
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override { return &Node; }
	// End of UAnimGraphNode_SkeletalControlBase interface
};
