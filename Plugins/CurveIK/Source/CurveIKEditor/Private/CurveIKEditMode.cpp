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

	CurveIKEditModeBase::EnterMode(InEditorNode, InRuntimeNode);
}

void FCurveIKEditMode::ExitMode()
{
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
		const float LineScale = 200;

		// P Vector
		PDI->DrawLine(
			P1,
			P2,
			FLinearColor::FromSRGBColor(FColor::Cyan),
			SDPG_Foreground
		);
		PDI->DrawPoint(P1, FColor::Cyan, 15, SDPG_Foreground);
		PDI->DrawPoint(P2, FColor::Cyan, 15, SDPG_Foreground);

		PDI->DrawLine(
			MidPoint,
			MidPoint + (CurveIKDebugData.HandleDir * LineScale),
			FLinearColor::FromSRGBColor(FColor::Magenta),
			SDPG_Foreground
		);

		if (CurveIKDebugData.ControlPoints.Num() >= 3)
		{
			FVector HandleEnd1 = CurveIKDebugData.ControlPoints[1];
			// Toggle between quadratic and cubic bezier
			FVector HandleEnd2 = CurveIKDebugData.ControlPoints[CurveIKDebugData.ControlPoints.Num() == 4 ? 2 : 1];
			PDI->DrawLine(
				P1,
				HandleEnd1,
				FLinearColor::FromSRGBColor(FColor::Red),
				SDPG_Foreground
			);
			PDI->DrawPoint(HandleEnd1, FLinearColor::FromSRGBColor(FColor::Red), 10, SDPG_Foreground);
			
			PDI->DrawLine(
				P2,
				HandleEnd2,
				FLinearColor::FromSRGBColor(FColor::Red),
				SDPG_Foreground
			);
			PDI->DrawPoint(HandleEnd2, FLinearColor::FromSRGBColor(FColor::Red), 10, SDPG_Foreground);
		}

		PDI->DrawLine(
			MidPoint,
			MidPoint + (CurveIKDebugData.RightVector * LineScale),
			FLinearColor::FromSRGBColor(FColor::Green),
			SDPG_Foreground
		);

		PDI->DrawLine(
			MidPoint,
			MidPoint + (CurveIKDebugData.UpVector * LineScale),
			FLinearColor::FromSRGBColor(FColor::Blue),
			SDPG_Foreground
		);
	
		for (int i = 1; i < RuntimeNode->Chain.Num(); i++)
		{
			FCurveIKChainLink ChainLink = RuntimeNode->Chain[i];
			FCurveIKChainLink LastChainLink = RuntimeNode->Chain[i - 1];
			if (RuntimeNode->bShowLinks)
			{				
				PDI->DrawLine(
					LastChainLink.Position,
					ChainLink.Position,
					FLinearColor::FromSRGBColor(FColor::Orange),
					SDPG_Foreground
				);
				PDI->DrawPoint(ChainLink.Position, FLinearColor::FromSRGBColor(FColor::Orange), 30, SDPG_Foreground);
			}
			if (RuntimeNode->bShowTangents)
			{
				PDI->DrawLine(
					ChainLink.Position,
					ChainLink.Position + (ChainLink.CurvePoint.Tangent * 200.f),
					FLinearColor::FromSRGBColor(FColor::Emerald),
					SDPG_Foreground
				);
			}
			if (RuntimeNode->bShowNormals)
			{				
				PDI->DrawLine(
					ChainLink.Position,
					ChainLink.Position + (ChainLink.CurvePoint.Normal * 200.f),
					FLinearColor::FromSRGBColor(FColor::Red),
					SDPG_Foreground
				);
			}
			if (RuntimeNode->bShowBoneDirection)
			{				
				PDI->DrawLine(
					ChainLink.Position,
					ChainLink.Position + (ChainLink.BoneDownVector * 200.f),
					FLinearColor::FromSRGBColor(FColor::Yellow),
					SDPG_Foreground
				);
			}
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