// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class CurvesIK_Sample : ModuleRules
{
	public CurvesIK_Sample(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "CurveIK" });

		PrivateDependencyModuleNames.AddRange(new string[] { "CurveIK" });
	}
}
