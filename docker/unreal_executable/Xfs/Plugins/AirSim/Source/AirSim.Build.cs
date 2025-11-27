// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

using UnrealBuildTool;
using System;
using System.IO;

public class AirSim : ModuleRules
{
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    private string AirLibPath
    {
        get { return Path.Combine(ModulePath, "AirLib"); }
    }
    private string AirSimPluginPath
    {
        get { return Directory.GetParent(ModulePath).FullName; }
    }
    private string ProjectBinariesPath
    {
        get
        {
            return Path.Combine(
                Directory.GetParent(AirSimPluginPath).Parent.FullName, "Binaries");
        }
    }
    private string AirSimPluginDependencyPath
    {
        get { return Path.Combine(AirSimPluginPath, "Dependencies"); }
    }

    private enum CompileMode
    {
        HeaderOnlyNoRpc,
        HeaderOnlyWithRpc,
        CppCompileNoRpc,
        CppCompileWithRpc
    }

    private void SetupCompileMode(CompileMode mode, ReadOnlyTargetRules Target)
    {
        LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");

        switch (mode)
        {
            case CompileMode.HeaderOnlyNoRpc:
                PublicDefinitions.Add("AIRLIB_HEADER_ONLY=1");
                PublicDefinitions.Add("AIRLIB_NO_RPC=1");
                AddLibDependency("AirLib", Path.Combine(AirLibPath, "lib"), "AirLib", Target, false);
                break;

            case CompileMode.HeaderOnlyWithRpc:
                PublicDefinitions.Add("AIRLIB_HEADER_ONLY=1");
                AddLibDependency("AirLib", Path.Combine(AirLibPath, "lib"), "AirLib", Target, false);
                LoadAirSimDependency(Target, "rpclib", "rpc");
                break;

            case CompileMode.CppCompileNoRpc:
                LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");
                PublicDefinitions.Add("AIRLIB_NO_RPC=1");
                break;

            case CompileMode.CppCompileWithRpc:
                LoadAirSimDependency(Target, "rpclib", "rpc");
                break;

            default:
                throw new System.Exception("CompileMode specified in plugin's Build.cs file is not recognized");
        }

    }

    public AirSim(ReadOnlyTargetRules Target) : base(Target)
    {
        //bEnforceIWYU = true; //to support 4.16
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ImageWrapper", "RenderCore", "RHI", "AssetRegistry", "PhysicsCore", "ChaosVehicles", "Landscape", "CinematicCamera" });
        PrivateDependencyModuleNames.AddRange(new string[] { "UMG", "Slate", "SlateCore", "RenderCore", "ChaosVehicles" });

        //suppress VC++ proprietary warnings
        PublicDefinitions.Add("_SCL_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("HMD_MODULE_INCLUDED=0");

        PublicIncludePaths.Add(Path.Combine(AirLibPath, "include"));
        PublicIncludePaths.Add(Path.Combine(AirLibPath, "deps", "eigen3"));

        // Platform-specific configuration with complete isolation
        AddOSLibDependencies(Target);

        SetupCompileMode(CompileMode.HeaderOnlyWithRpc, Target);
    }

    private void AddOSLibDependencies(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            ConfigureWindowsPlatform(Target);
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            ConfigureLinuxPlatform(Target);
        }
        else if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            ConfigureMacPlatform(Target);
        }
    }

    private void ConfigureWindowsPlatform(ReadOnlyTargetRules Target)
    {
        // Windows-specific libraries and configuration
        // for SHGetFolderPath.
        PublicAdditionalLibraries.Add("Shell32.lib");

        //for joystick support
        PublicAdditionalLibraries.Add("dinput8.lib");
        PublicAdditionalLibraries.Add("dxguid.lib");

        // Windows-specific definitions (if any needed for AirSim)
        // Keep Windows builds completely isolated from Linux settings
    }

    private void ConfigureLinuxPlatform(ReadOnlyTargetRules Target)
    {
        // CRITICAL: Only configure Epic toolchain if we're actually in a Linux environment
        // with Epic's UE5 installation - prevents Windows contamination

        // Epic's UE5 bundled libc++ paths (Linux Container Only)
        string EpicLibCppInclude = "/home/ue4/UnrealEngine/Engine/Source/ThirdParty/Unix/LibCxx/include/c++/v1";
        string EpicLibCppLib = "/home/ue4/UnrealEngine/Engine/Source/ThirdParty/Unix/LibCxx/lib/Unix/x86_64-unknown-linux-gnu";

        // Defensive check: Only apply Epic configuration if paths actually exist
        // This prevents Windows builds from being affected by non-existent Linux paths
        if (System.IO.Directory.Exists(EpicLibCppInclude) && System.IO.Directory.Exists(EpicLibCppLib))
        {
            ConfigureEpicToolchain(Target, EpicLibCppInclude, EpicLibCppLib);
        }
        else
        {
            // Fallback: Use standard Linux configuration without Epic-specific overrides
            ConfigureStandardLinux(Target);
        }
    }

    private void ConfigureMacPlatform(ReadOnlyTargetRules Target)
    {
        // Mac-specific configuration (placeholder for future use)
        // Keep Mac builds isolated from both Windows and Linux settings
    }

    private void ConfigureEpicToolchain(ReadOnlyTargetRules Target, string EpicLibCppInclude, string EpicLibCppLib)
    {
        // Epic's UE5 bundled toolchain configuration - LINUX ONLY
        // This method is only called when Epic paths exist (defensive check in ConfigureLinuxPlatform)

        // Platform-specific definitions (Linux only)
        PublicDefinitions.Add("_LIBCPP_ABI_UNSTABLE=1");
        PublicDefinitions.Add("_LIBCPP_DISABLE_AVAILABILITY=1");
        PublicDefinitions.Add("_GLIBCXX_USE_CXX11_ABI=0");

        // Add Epic's libc++ include path with high priority (Linux only)
        PublicSystemIncludePaths.Add(EpicLibCppInclude);

        // Explicit Epic libc++ library paths and linking (Linux only)
        PublicSystemLibraryPaths.Add(EpicLibCppLib);
        PublicAdditionalLibraries.Add("c++");
        PublicAdditionalLibraries.Add("c++abi");

        // Note: Compiler and linker flags (stdlib, nostdinc++, etc.) are set globally
        // in the Docker container environment via CXXFLAGS and LDFLAGS.
        // UE5 removed PublicCompileEnvironment and PublicLinkEnvironment API.
        // The definitions and library paths above provide the necessary UE5 build configuration.
    }

    private void ConfigureStandardLinux(ReadOnlyTargetRules Target)
    {
        // Standard Linux configuration when Epic toolchain is not available
        // This provides a fallback that doesn't require Epic-specific paths

        // Basic Linux-specific definitions
        PublicDefinitions.Add("LINUX_STANDARD_BUILD=1");

        // Standard Linux libraries (if needed)
        // PublicAdditionalLibraries.Add("pthread");
        // PublicAdditionalLibraries.Add("dl");
    }

    static void CopyFileIfNewer(string srcFilePath, string destFolder)
    {
        FileInfo srcFile = new FileInfo(srcFilePath);
        FileInfo destFile = new FileInfo(Path.Combine(destFolder, srcFile.Name));
        if (!destFile.Exists || srcFile.LastWriteTime > destFile.LastWriteTime)
        {
            srcFile.CopyTo(destFile.FullName, true);
        }
        //else skip
    }

    private bool LoadAirSimDependency(ReadOnlyTargetRules Target, string LibName, string LibFileName)
    {
        string LibrariesPath = Path.Combine(AirLibPath, "deps", LibName, "lib");
        return AddLibDependency(LibName, LibrariesPath, LibFileName, Target, true);
    }

    private bool AddLibDependency(string LibName, string LibPath, string LibFileName, ReadOnlyTargetRules Target, bool IsAddLibInclude)
    {
        string PlatformString = (Target.Platform == UnrealTargetPlatform.Win64 || Target.Platform == UnrealTargetPlatform.Mac) ? "x64" : "x86";
        string ConfigurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";
        bool isLibrarySupported = false;


        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            isLibrarySupported = true;

            PublicAdditionalLibraries.Add(Path.Combine(LibPath, PlatformString, ConfigurationString, LibFileName + ".lib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac)
        {
            isLibrarySupported = true;
            PublicAdditionalLibraries.Add(Path.Combine(LibPath, "lib" + LibFileName + ".a"));
        }

        if (isLibrarySupported && IsAddLibInclude)
        {
            // Include path
            PublicIncludePaths.Add(Path.Combine(AirLibPath, "deps", LibName, "include"));
        }
        PublicDefinitions.Add(string.Format("WITH_" + LibName.ToUpper() + "_BINDING={0}", isLibrarySupported ? 1 : 0));

        return isLibrarySupported;
    }
}