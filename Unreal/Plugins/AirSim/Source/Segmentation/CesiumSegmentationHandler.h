#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "Engine/World.h"
#include "Components/StaticMeshComponent.h"
#include "CesiumSegmentationCapture.h"
#include "CesiumSegmentationHandler.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnCesiumTileLoaded, AActor*, TilesetActor, const TArray<UStaticMeshComponent*>&, NewMeshes);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnCesiumTileUnloaded, AActor*, TilesetActor, const TArray<UStaticMeshComponent*>&, RemovedMeshes);

USTRUCT(BlueprintType)
struct AIRSIM_API FCesiumTileSegmentationRule
{
    GENERATED_BODY()

    // Pattern to match against tile names
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString TileNamePattern;

    // Whether the pattern is a regex
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIsRegexPattern;

    // Segmentation ID to assign
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 SegmentationID;

    // Priority for rule application (higher priority = applied first)
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 Priority;

    // Whether this rule is enabled
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bEnabled;

    FCesiumTileSegmentationRule()
    {
        TileNamePattern = TEXT("");
        bIsRegexPattern = false;
        SegmentationID = 1;
        Priority = 0;
        bEnabled = true;
    }
};

USTRUCT(BlueprintType)
struct AIRSIM_API FCesiumSegmentationSettings
{
    GENERATED_BODY()

    // Auto-assign segmentation IDs to new tiles
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bAutoAssignTileIDs;

    // Range for auto-assigned tile IDs
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 TileIDRangeStart;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 TileIDRangeEnd;

    // Default segmentation ID for terrain/ground tiles
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 DefaultTerrainID;

    // Default segmentation ID for building tiles
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 DefaultBuildingID;

    // Rules for automatic segmentation assignment
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FCesiumTileSegmentationRule> SegmentationRules;

    // Whether to process tiles immediately on load
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bProcessTilesOnLoad;

    // Whether to cache tile segmentation data
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bCacheTileData;

    FCesiumSegmentationSettings()
    {
        bAutoAssignTileIDs = true;
        TileIDRangeStart = 100;
        TileIDRangeEnd = 199;
        DefaultTerrainID = 50;
        DefaultBuildingID = 75;
        bProcessTilesOnLoad = true;
        bCacheTileData = true;
    }
};

UCLASS(BlueprintType, Blueprintable)
class AIRSIM_API UCesiumSegmentationHandler : public UObject
{
    GENERATED_BODY()

public:
    UCesiumSegmentationHandler();

    // Initialize the handler with a segmentation capture system
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    bool Initialize(UCesiumSegmentationCapture* InSegmentationCapture, UWorld* InWorld);

    // Register callbacks for Cesium tile loading/unloading
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    void RegisterCesiumCallbacks();

    // Unregister callbacks
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    void UnregisterCesiumCallbacks();

    // Manually process all existing Cesium tiles
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    void ProcessAllExistingTiles();

    // Add a segmentation rule
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    void AddSegmentationRule(const FCesiumTileSegmentationRule& Rule);

    // Remove a segmentation rule by pattern
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    bool RemoveSegmentationRule(const FString& TileNamePattern);

    // Clear all segmentation rules
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    void ClearAllSegmentationRules();

    // Get current segmentation settings
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    FCesiumSegmentationSettings GetSegmentationSettings() const { return Settings; }

    // Update segmentation settings
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    void UpdateSegmentationSettings(const FCesiumSegmentationSettings& NewSettings);

    // Get statistics about processed tiles
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    int32 GetProcessedTileCount() const { return ProcessedTiles.Num(); }

    // Get list of all processed tile names
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    TArray<FString> GetProcessedTileNames() const;

    // Force refresh of all Cesium tile segmentation
    UFUNCTION(BlueprintCallable, Category = "Cesium Segmentation")
    void RefreshAllCesiumTileSegmentation();

    // Events
    UPROPERTY(BlueprintAssignable, Category = "Cesium Segmentation")
    FOnCesiumTileLoaded OnCesiumTileLoaded;

    UPROPERTY(BlueprintAssignable, Category = "Cesium Segmentation")
    FOnCesiumTileUnloaded OnCesiumTileUnloaded;

protected:
    // Handle when new Cesium tiles are loaded
    UFUNCTION()
    void HandleCesiumTileLoaded(AActor* TilesetActor, const TArray<UStaticMeshComponent*>& NewMeshes);

    // Handle when Cesium tiles are unloaded
    UFUNCTION()
    void HandleCesiumTileUnloaded(AActor* TilesetActor, const TArray<UStaticMeshComponent*>& RemovedMeshes);

    // Core segmentation system reference
    UPROPERTY()
    UCesiumSegmentationCapture* SegmentationCapture;

    // World reference
    UPROPERTY()
    UWorld* World;

    // Settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
    FCesiumSegmentationSettings Settings;

    // Tracking data
    UPROPERTY()
    TMap<FString, uint8> ProcessedTiles; // TileName -> SegmentationID

    UPROPERTY()
    TArray<AActor*> CesiumTilesetActors;

    UPROPERTY()
    uint8 NextAutoAssignID;

private:
    // Internal processing methods
    void ApplySegmentationToCesiumMeshes(const TArray<UStaticMeshComponent*>& Meshes, AActor* TilesetActor);
    uint8 DetermineSegmentationIDForCesiumMesh(UStaticMeshComponent* Mesh, const FString& TileName);
    uint8 GetNextAutoAssignID();
    
    // Rule processing
    FCesiumTileSegmentationRule* FindMatchingRule(const FString& TileName);
    bool MatchesRule(const FString& TileName, const FCesiumTileSegmentationRule& Rule);
    
    // Cesium-specific utilities
    bool IsCesiumTilesetActor(AActor* Actor);
    FString GetTileNameFromMesh(UStaticMeshComponent* Mesh);
    TArray<UStaticMeshComponent*> GetMeshComponentsFromTileset(AActor* TilesetActor);
    
    // Tile classification
    bool IsTerrainTile(const FString& TileName);
    bool IsBuildingTile(const FString& TileName);
    bool IsVegetationTile(const FString& TileName);
    
    // Callback management
    bool bCallbacksRegistered;
    
    // Auto ID management
    void InitializeAutoIDRange();
    bool IsAutoIDInRange(uint8 ID) const;
    
    // Caching
    UPROPERTY()
    TMap<FString, FCesiumTileSegmentationRule> RuleCache;
    
    void UpdateRuleCache();
    
    // Debug and logging
    void LogTileProcessing(const FString& TileName, uint8 AssignedID, const FString& Reason);
    void LogHandlerState();
};