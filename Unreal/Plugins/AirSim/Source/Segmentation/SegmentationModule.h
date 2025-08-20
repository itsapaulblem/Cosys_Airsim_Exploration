#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

/**
 * The public interface to this module
 */
class ISegmentationModule : public IModuleInterface
{
public:
    /**
     * Singleton-like access to this module's interface. This is just for convenience!
     * Beware of calling this during the shutdown phase, though. Your module might have been unloaded already.
     *
     * @return Returns singleton instance, loading the module on demand if needed
     */
    static inline ISegmentationModule& Get()
    {
        return FModuleManager::LoadModuleChecked<ISegmentationModule>("Segmentation");
    }

    /**
     * Checks to see if this module is loaded and ready. It is only valid to call Get() if IsAvailable() returns true.
     *
     * @return True if the module is loaded and ready to use
     */
    static inline bool IsAvailable()
    {
        return FModuleManager::Get().IsModuleLoaded("Segmentation");
    }
};

/**
 * The Segmentation module implementation
 */
class FSegmentationModule : public ISegmentationModule
{
public:
    /** IModuleInterface implementation */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

private:
    /** Handle to the test dll we will load */
    void* ExampleLibraryHandle;
};