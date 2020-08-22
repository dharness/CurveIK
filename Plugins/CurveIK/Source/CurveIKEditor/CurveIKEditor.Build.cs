// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class CurveIKEditor : ModuleRules
{
	public CurveIKEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PrivateDependencyModuleNames.AddRange(new string[] {
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"CurveIK"
		});

		PrivateDependencyModuleNames.AddRange(new string[] {
			"UnrealED",
			"AnimGraph",
			"AnimGraphRuntime",
			"BlueprintGraph",
			"Persona",
			"SlateCore"
		});
	}
}
