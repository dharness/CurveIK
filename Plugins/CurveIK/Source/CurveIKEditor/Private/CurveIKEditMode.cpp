// Copyright Epic Games, Inc. All Rights Reserved.

#include "CurveIKEditMode.h"
#include "AnimNode_CurveIK.h"
#include "AnimGraphNode_CurveIK.h"
#include "IPersonaPreviewScene.h"
#include "Animation/DebugSkelMeshComponent.h"


void FCurveIKEditMode::EnterMode(class UAnimGraphNode_Base* InEditorNode, struct FAnimNode_Base* InRuntimeNode)
{
	RuntimeNode = static_cast<FAnimNode_CurveIK*>(InRuntimeNode);
	GraphNode = CastChecked<UAnimGraphNode_CurveIK>(InEditorNode);
	UE_LOG(LogTemp, Warning, TEXT("EnterMode"));

	CurveIKEditModeBase::EnterMode(InEditorNode, InRuntimeNode);
}

void FCurveIKEditMode::ExitMode()
{
	UE_LOG(LogTemp, Warning, TEXT("ExitMode"));
	RuntimeNode = nullptr;
	GraphNode = nullptr;

	CurveIKEditModeBase::ExitMode();
}

FVector FCurveIKEditMode::GetWidgetLocation() const
{
	EBoneControlSpace Space = RuntimeNode->EffectorLocationSpace;
	FVector Location = RuntimeNode->EffectorLocation;
	FBoneSocketTarget Target = RuntimeNode->EffectorTarget;

	USkeletalMeshComponent* SkelComp = GetAnimPreviewScene().GetPreviewMeshComponent();
	return Location;
}

FWidget::EWidgetMode FCurveIKEditMode::GetWidgetMode() const
{
	USkeletalMeshComponent* SkelComp = GetAnimPreviewScene().GetPreviewMeshComponent();
	int32 TipBoneIndex = SkelComp->GetBoneIndex(RuntimeNode->TipBone.BoneName);
	int32 RootBoneIndex = SkelComp->GetBoneIndex(RuntimeNode->RootBone.BoneName);

	if (TipBoneIndex != INDEX_NONE && RootBoneIndex != INDEX_NONE)
	{
		return FWidget::WM_Translate;
	}
	return FWidget::WM_None;
}

void FCurveIKEditMode::DoTranslation(FVector& InTranslation)
{
	USkeletalMeshComponent* SkelComp = GetAnimPreviewScene().GetPreviewMeshComponent();

	FVector Offset = ConvertCSVectorToBoneSpace(SkelComp, InTranslation, RuntimeNode->ForwardedPose, RuntimeNode->EffectorTarget, GraphNode->Node.EffectorLocationSpace);

	RuntimeNode->EffectorLocation += Offset;
	GraphNode->Node.EffectorLocation = RuntimeNode->EffectorLocation;
	GraphNode->SetDefaultValue(GET_MEMBER_NAME_STRING_CHECKED(FAnimNode_CurveIK, EffectorLocation), RuntimeNode->EffectorLocation);
}

void FCurveIKEditMode::Render(const FSceneView* View, FViewport* Viewport, FPrimitiveDrawInterface* PDI)
{
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	// draw line from root bone to tip bone if available
	if (RuntimeNode && RuntimeNode->bEnableDebugDraw)
	{
		USkeletalMeshComponent* SkelComp = GetAnimPreviewScene().GetPreviewMeshComponent();
		FTransform CompToWorld = SkelComp->GetComponentToWorld();
		FCurveIKDebugData CurveIKDebugData = RuntimeNode->CurveIKDebugData;
		const FVector MidPoint = CurveIKDebugData.P1 + (CurveIKDebugData.P2 / 2.0);
		const FVector P1 = CurveIKDebugData.P1;
		const FVector P2 = CurveIKDebugData.P2;
		const FVector ControlPoint = CurveIKDebugData.ControlPoint;

		const float LineScale = 30;
		PDI->DrawPoint(P1, FLinearColor::Blue, 15, SDPG_Foreground);
		PDI->DrawPoint(P2, FLinearColor::Blue, 15, SDPG_Foreground);
		PDI->DrawPoint(ControlPoint, FLinearColor::Red, 15, SDPG_Foreground);


		// P Vector
		PDI->DrawLine(
			P1,
			P2,
			FLinearColor::FromSRGBColor(FColor::Cyan),
			SDPG_Foreground
		);

		PDI->DrawLine(
			MidPoint,
			MidPoint + (CurveIKDebugData.ControlVector * LineScale),
			FLinearColor::FromSRGBColor(FColor::Red),
			SDPG_Foreground
		);

		const auto MidPointOriginal = (P1 + (CurveIKDebugData.UpVector * LineScale)) / 2;
		PDI->DrawLine(
			MidPointOriginal,
			MidPointOriginal + (CurveIKDebugData.RightVector.GetSafeNormal() * LineScale),
			FLinearColor::FromSRGBColor(FColor::Orange),
			SDPG_Foreground
		);

		PDI->DrawLine(
			P1,
			P1 + (CurveIKDebugData.UpVector * LineScale),
			FLinearColor::FromSRGBColor(FColor::Orange),
			SDPG_Foreground
		);
	
		for (int i = 1; i < RuntimeNode->Chain.Num(); i++)
		{
			FCurveIKChainLink ChainLink = RuntimeNode->Chain[i];
			FCurveIKChainLink LastChainLink = RuntimeNode->Chain[i - 1];
			PDI->DrawLine(
				LastChainLink.Position,
				ChainLink.Position,
				FLinearColor::FromSRGBColor(FColor::Orange),
				SDPG_Foreground
			);
			PDI->DrawLine(
				ChainLink.Position,
				ChainLink.Position + (ChainLink.ForwardVector * 200.f),
				FLinearColor::FromSRGBColor(FColor::Emerald),
				SDPG_Foreground
			);
			PDI->DrawPoint(ChainLink.Position, FLinearColor::FromSRGBColor(FColor::Orange), 30, SDPG_Foreground);
			FCurveIKChainLink ParentChainLink = RuntimeNode->Chain[i - 1];
		}

		auto CachePoints = CurveIKDebugData.CurveCache.GetPoints();
		for(int i = 0; i < CachePoints.Num(); i++)
		{
			auto Point = CachePoints[i];
			PDI->DrawPoint(Point, FLinearColor::FromSRGBColor(FColor::Purple), 10, SDPG_Foreground);
			if(i > 0)
			{
				auto PrevPoint = CachePoints[i - 1];
				PDI->DrawLine(
					PrevPoint,
					Point,
					FLinearColor::FromSRGBColor(FColor::Purple),
					SDPG_Foreground
				);
			}
		}
	}
#endif // #if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
}