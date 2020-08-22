#include "CurveIKEditor.h"
#include "EditorModeRegistry.h"
#include "Modules/ModuleManager.h"
#include "Modules/ModuleInterface.h"
#include "Textures/SlateIcon.h"
#include "CurveIKEditMode.h"
#include "CurveIKEditModes.h"


#define LOCTEXT_NAMESPACE "FCurveIKEditorModule"

void FCurveIKEditorModule::StartupModule()
{
	FEditorModeRegistry::Get().RegisterMode<FCurveIKEditMode>(CurveIKEditModes::CurveIK, FText::FromString("CurveIKEditor"), FSlateIcon(), false);
}

void FCurveIKEditorModule::ShutdownModule()
{
	FEditorModeRegistry::Get().UnregisterMode(CurveIKEditModes::CurveIK);
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FCurveIKEditorModule, CurveIKEditor)