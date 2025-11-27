using UnrealBuildTool;

public class Segmentation : ModuleRules
{
    public Segmentation(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicIncludePaths.AddRange(
            new string[] {
                // ... add public include paths required here ...
            }
        );

        PrivateIncludePaths.AddRange(
            new string[] {
                // ... add other private include paths required here ...
            }
        );

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Engine",
                "RenderCore",
                "RHI",
                "Slate",
                "SlateCore",
                "UMG",
                "UnrealEd",
                "ToolMenus",
                "Projects",
                "AirSim"
                // ... add other public dependencies that you statically link with here ...
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Projects",
                "InputCore",
                "UnrealEd",
                "LevelEditor",
                "CoreUObject",
                "Engine",
                "Slate",
                "SlateCore",
                "EditorStyle",
                "EditorWidgets",
                "DesktopWidgets",
                "PropertyEditor",
                "SharedSettingsWidgets",
                "SequencerWidgets",
                "BlueprintGraph",
                "AnimGraph",
                "ComponentVisualizers",
                "RenderCore",
                "RHI",
                "ShaderCore",
                "UtilityShaders",
                "Renderer",
                "MaterialShaderQualitySettings",
                "KismetCompiler",
                "ToolMenus",
                "KismetWidgets",
                "PropertyEditor",
                "BlueprintGraph",
                "GraphColor",
                "ContentBrowser",
                "WorkspaceMenuStructure",
                "EditorWidgets",
                "EngineSettings",
                "Json",
                "JsonUtilities"
                // ... add private dependencies that you statically link with here ...
            }
        );

        DynamicallyLoadedModuleNames.AddRange(
            new string[]
            {
                // ... add any modules that your module loads dynamically here ...
            }
        );

        // Additional settings for segmentation functionality
        bEnableExceptions = true;
        
        if (Target.bBuildEditor == true)
        {
            PublicDependencyModuleNames.AddRange(
                new string[]
                {
                    "UnrealEd",
                    "Slate",
                    "SlateCore",
                    "EditorStyle",
                    "EditorWidgets",
                    "PropertyEditor",
                    "SharedSettingsWidgets",
                    "SequencerWidgets"
                }
            );
        }

        // Platform specific configurations
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // Windows specific settings
            PublicAdditionalLibraries.Add("opengl32.lib");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            // Linux specific settings
            PublicAdditionalLibraries.Add("GL");
        }
        else if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            // Mac specific settings
            PublicFrameworks.AddRange(new string[] { "OpenGL" });
        }

        // Additional defines
        PublicDefinitions.Add("WITH_SEGMENTATION=1");
        
        if (Target.Configuration != UnrealTargetConfiguration.Shipping)
        {
            PublicDefinitions.Add("SEGMENTATION_DEBUG=1");
        }
    }
}