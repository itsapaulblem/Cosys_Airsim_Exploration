#include "SegmentationModule.h"
#include "Core.h"
#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"

#define LOCTEXT_NAMESPACE "FSegmentationModule"

void FSegmentationModule::StartupModule()
{
    // This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module

    // Get the base directory of this plugin
    FString BaseDir = IPluginManager::Get().FindPlugin("AirSim")->GetBaseDir();

    // Add on the relative location of the third party dll and load it
    FString LibraryPath;
#if PLATFORM_WINDOWS
    LibraryPath = FPaths::Combine(*BaseDir, TEXT("Binaries/ThirdParty/SegmentationLibrary/Win64/ExampleLibrary.dll"));
#elif PLATFORM_MAC
    LibraryPath = FPaths::Combine(*BaseDir, TEXT("Binaries/ThirdParty/SegmentationLibrary/Mac/libExampleLibrary.dylib"));
#elif PLATFORM_LINUX
    LibraryPath = FPaths::Combine(*BaseDir, TEXT("Binaries/ThirdParty/SegmentationLibrary/Linux/x86_64-unknown-linux-gnu/libExampleLibrary.so"));
#endif

    ExampleLibraryHandle = nullptr;
    if (!LibraryPath.IsEmpty())
    {
        ExampleLibraryHandle = FPlatformProcess::GetDllHandle(*LibraryPath);

        if (ExampleLibraryHandle)
        {
            UE_LOG(LogTemp, Log, TEXT("Segmentation library loaded successfully"));
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Failed to load segmentation library"));
        }
    }

    UE_LOG(LogTemp, Log, TEXT("Segmentation module started"));
}

void FSegmentationModule::ShutdownModule()
{
    // This function may be called during shutdown to clean up your module. For modules that support dynamic reloading,
    // we call this function before unloading the module.

    // Free the dll handle
    if (ExampleLibraryHandle)
    {
        FPlatformProcess::FreeDllHandle(ExampleLibraryHandle);
        ExampleLibraryHandle = nullptr;
    }

    UE_LOG(LogTemp, Log, TEXT("Segmentation module shut down"));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FSegmentationModule, Segmentation)