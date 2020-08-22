// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CurveIKEditMode.h"
#include "CurveIKEditModeBase.h"

class FCurveIKEditMode : public CurveIKEditModeBase
{
public:
	/** IAnimNodeEditMode interface */
	virtual void EnterMode(class UAnimGraphNode_Base* InEditorNode, struct FAnimNode_Base* InRuntimeNode) override;
	virtual void ExitMode() override;
	virtual FVector GetWidgetLocation() const override;
	virtual FWidget::EWidgetMode GetWidgetMode() const override;
	virtual bool SetWidgetMode(FWidget::EWidgetMode InWidgetMode) override { return true; };
	virtual void DoTranslation(FVector& InTranslation) override;
	virtual void Render(const FSceneView* View, FViewport* Viewport, FPrimitiveDrawInterface* PDI) override;

private:
	struct FAnimNode_CurveIK* RuntimeNode;
	class UAnimGraphNode_CurveIK* GraphNode;
};
