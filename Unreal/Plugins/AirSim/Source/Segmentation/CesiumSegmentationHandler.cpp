#include "CesiumSegmentationHandler.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "Components/StaticMeshComponent.h"
#include "Regex.h"
#include "Kismet/KismetStringLibrary.h"

DEFINE_LOG_CATEGORY_EXTERN(LogCesiumSegmentationHandler, Log, All);
DEFINE_LOG_CATEGORY(LogCesiumSegmentationHandler);

UCesiumSegmentationHandler::UCesiumSegmentationHandler()
{
    SegmentationCapture = nullptr;
    World = nullptr;
    bCallbacksRegistered = false;
    NextAutoAssignID = 100; // Default starting ID
    
    // Initialize default settings
    Settings = FCesiumSegmentationSettings();
    InitializeAutoIDRange();
}

bool UCesiumSegmentationHandler::Initialize(UCesiumSegmentationCapture* InSegmentationCapture, UWorld* InWorld)
{
    if (!InSegmentationCapture || !InWorld)
    {
        UE_LOG(LogCesiumSegmentationHandler, Error, TEXT("Invalid parameters provided to Initialize"));
        return false;
    }

    SegmentationCapture = InSegmentationCapture;
    World = InWorld;
    
    InitializeAutoIDRange();
    UpdateRuleCache();
    
    // Find existing Cesium tileset actors
    ProcessAllExistingTiles();
    
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Cesium segmentation handler initialized successfully"));
    return true;
}

void UCesiumSegmentationHandler::RegisterCesiumCallbacks()
{
    if (bCallbacksRegistered || !World)
    {
        return;
    }

    // Note: In a real implementation, you would bind to actual Cesium events
    // For now, we'll set up a timer to periodically check for new tiles
    FTimerHandle TimerHandle;
    World->GetTimerManager().SetTimer(TimerHandle, this, 
        &UCesiumSegmentationHandler::ProcessAllExistingTiles, 5.0f, true);

    bCallbacksRegistered = true;
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Cesium callbacks registered"));
}

void UCesiumSegmentationHandler::UnregisterCesiumCallbacks()
{
    if (!bCallbacksRegistered)
    {
        return;
    }

    // Clear any active timers
    if (World)
    {
        World->GetTimerManager().ClearAllTimersForObject(this);
    }

    bCallbacksRegistered = false;
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Cesium callbacks unregistered"));
}

void UCesiumSegmentationHandler::ProcessAllExistingTiles()
{
    if (!World || !SegmentationCapture)
    {
        return;
    }

    CesiumTilesetActors.Empty();
    
    // Find all Cesium tileset actors in the world
    for (TActorIterator<AActor> ActorIterator(World); ActorIterator; ++ActorIterator)
    {
        AActor* Actor = *ActorIterator;
        if (Actor && IsCesiumTilesetActor(Actor))
        {
            CesiumTilesetActors.AddUnique(Actor);
            
            // Get mesh components from this tileset
            TArray<UStaticMeshComponent*> MeshComponents = GetMeshComponentsFromTileset(Actor);
            
            if (MeshComponents.Num() > 0)
            {
                ApplySegmentationToCesiumMeshes(MeshComponents, Actor);
                
                // Broadcast event
                OnCesiumTileLoaded.Broadcast(Actor, MeshComponents);
            }
        }
    }

    UE_LOG(LogCesiumSegmentationHandler, Verbose, 
           TEXT("Processed %d Cesium tileset actors"), CesiumTilesetActors.Num());
}

void UCesiumSegmentationHandler::HandleCesiumTileLoaded(AActor* TilesetActor, const TArray<UStaticMeshComponent*>& NewMeshes)
{
    if (!TilesetActor || NewMeshes.Num() == 0)
    {
        return;
    }

    ApplySegmentationToCesiumMeshes(NewMeshes, TilesetActor);
    
    UE_LOG(LogCesiumSegmentationHandler, Log, 
           TEXT("Handled tile loaded event for %d meshes from tileset: %s"), 
           NewMeshes.Num(), *TilesetActor->GetName());
}

void UCesiumSegmentationHandler::HandleCesiumTileUnloaded(AActor* TilesetActor, const TArray<UStaticMeshComponent*>& RemovedMeshes)
{
    if (!TilesetActor)
    {
        return;
    }

    // Remove processed tile entries for unloaded meshes
    for (UStaticMeshComponent* Mesh : RemovedMeshes)
    {
        if (Mesh)
        {
            FString TileName = GetTileNameFromMesh(Mesh);
            ProcessedTiles.Remove(TileName);
        }
    }

    UE_LOG(LogCesiumSegmentationHandler, Log, 
           TEXT("Handled tile unloaded event for %d meshes from tileset: %s"), 
           RemovedMeshes.Num(), *TilesetActor->GetName());
}

void UCesiumSegmentationHandler::ApplySegmentationToCesiumMeshes(const TArray<UStaticMeshComponent*>& Meshes, AActor* TilesetActor)
{
    if (!SegmentationCapture)
    {
        return;
    }

    for (UStaticMeshComponent* Mesh : Meshes)
    {
        if (!Mesh)
        {
            continue;
        }

        FString TileName = GetTileNameFromMesh(Mesh);
        
        // Skip if already processed (unless settings say to re-process)
        if (ProcessedTiles.Contains(TileName) && Settings.bCacheTileData)
        {
            continue;
        }

        // Determine segmentation ID for this mesh
        uint8 SegmentationID = DetermineSegmentationIDForCesiumMesh(Mesh, TileName);
        
        // Apply segmentation to the mesh's owner actor
        AActor* MeshOwner = Mesh->GetOwner();
        if (MeshOwner)
        {
            bool bSuccess = SegmentationCapture->SetObjectSegmentationID(MeshOwner, SegmentationID);
            if (bSuccess)
            {
                ProcessedTiles.Add(TileName, SegmentationID);
                LogTileProcessing(TileName, SegmentationID, TEXT("Applied via rule or auto-assignment"));
            }
        }
    }
}

uint8 UCesiumSegmentationHandler::DetermineSegmentationIDForCesiumMesh(UStaticMeshComponent* Mesh, const FString& TileName)
{
    if (!Mesh)
    {
        return Settings.DefaultTerrainID;
    }

    // First, check if there's a matching rule
    FCesiumTileSegmentationRule* MatchingRule = FindMatchingRule(TileName);
    if (MatchingRule && MatchingRule->bEnabled)
    {
        return MatchingRule->SegmentationID;
    }

    // If no rule matches, use heuristics based on tile type
    if (IsTerrainTile(TileName))
    {
        return Settings.DefaultTerrainID;
    }
    else if (IsBuildingTile(TileName))
    {
        return Settings.DefaultBuildingID;
    }
    else if (IsVegetationTile(TileName))
    {
        return Settings.DefaultTerrainID + 1; // Slightly different from terrain
    }

    // Default case - use auto-assignment if enabled
    if (Settings.bAutoAssignTileIDs)
    {
        return GetNextAutoAssignID();
    }

    return Settings.DefaultTerrainID;
}

FCesiumTileSegmentationRule* UCesiumSegmentationHandler::FindMatchingRule(const FString& TileName)
{
    // Sort rules by priority (higher priority first)
    TArray<FCesiumTileSegmentationRule*> SortedRules;
    for (FCesiumTileSegmentationRule& Rule : Settings.SegmentationRules)
    {
        if (Rule.bEnabled)
        {
            SortedRules.Add(&Rule);
        }
    }

    SortedRules.Sort([](const FCesiumTileSegmentationRule& A, const FCesiumTileSegmentationRule& B) {
        return A.Priority > B.Priority;
    });

    // Find the first matching rule
    for (FCesiumTileSegmentationRule* Rule : SortedRules)
    {
        if (MatchesRule(TileName, *Rule))
        {
            return Rule;
        }
    }

    return nullptr;
}

bool UCesiumSegmentationHandler::MatchesRule(const FString& TileName, const FCesiumTileSegmentationRule& Rule)
{
    if (Rule.bIsRegexPattern)
    {
        const FRegexPattern RegexPattern(Rule.TileNamePattern);
        FRegexMatcher RegexMatcher(RegexPattern, TileName);
        return RegexMatcher.FindNext();
    }
    else
    {
        return TileName.Contains(Rule.TileNamePattern, ESearchCase::IgnoreCase);
    }
}

uint8 UCesiumSegmentationHandler::GetNextAutoAssignID()
{
    uint8 CurrentID = NextAutoAssignID;
    
    // Increment for next time, wrapping around if necessary
    NextAutoAssignID++;
    if (NextAutoAssignID > Settings.TileIDRangeEnd)
    {
        NextAutoAssignID = Settings.TileIDRangeStart;
    }

    return CurrentID;
}

bool UCesiumSegmentationHandler::IsCesiumTilesetActor(AActor* Actor)
{
    if (!Actor)
    {
        return false;
    }

    FString ClassName = Actor->GetClass()->GetName();
    FString ActorName = Actor->GetName();

    // Check for common Cesium actor patterns
    return ClassName.Contains(TEXT("Cesium")) || 
           ClassName.Contains(TEXT("Tileset")) || 
           ClassName.Contains(TEXT("3DTile")) ||
           ActorName.Contains(TEXT("Cesium")) ||
           ActorName.Contains(TEXT("Tileset"));
}

FString UCesiumSegmentationHandler::GetTileNameFromMesh(UStaticMeshComponent* Mesh)
{
    if (!Mesh)
    {
        return TEXT("Unknown");
    }

    // Try to get a meaningful name from the mesh or its owner
    AActor* Owner = Mesh->GetOwner();
    if (Owner)
    {
        return Owner->GetName();
    }

    return Mesh->GetName();
}

TArray<UStaticMeshComponent*> UCesiumSegmentationHandler::GetMeshComponentsFromTileset(AActor* TilesetActor)
{
    TArray<UStaticMeshComponent*> MeshComponents;
    
    if (!TilesetActor)
    {
        return MeshComponents;
    }

    // Get all static mesh components from the actor and its children
    TArray<UStaticMeshComponent*> DirectComponents;
    TilesetActor->GetComponents<UStaticMeshComponent>(DirectComponents);
    MeshComponents.Append(DirectComponents);

    // Also check child actors (Cesium tiles are often hierarchical)
    TArray<AActor*> AttachedActors;
    TilesetActor->GetAttachedActors(AttachedActors);
    
    for (AActor* ChildActor : AttachedActors)
    {
        if (ChildActor)
        {
            TArray<UStaticMeshComponent*> ChildComponents;
            ChildActor->GetComponents<UStaticMeshComponent>(ChildComponents);
            MeshComponents.Append(ChildComponents);
        }
    }

    return MeshComponents;
}

bool UCesiumSegmentationHandler::IsTerrainTile(const FString& TileName)
{
    // Common patterns for terrain tiles
    TArray<FString> TerrainKeywords = {
        TEXT("terrain"), TEXT("ground"), TEXT("surface"), TEXT("dem"), 
        TEXT("elevation"), TEXT("heightmap"), TEXT("landscape")
    };

    for (const FString& Keyword : TerrainKeywords)
    {
        if (TileName.Contains(Keyword, ESearchCase::IgnoreCase))
        {
            return true;
        }
    }

    return false;
}

bool UCesiumSegmentationHandler::IsBuildingTile(const FString& TileName)
{
    // Common patterns for building tiles
    TArray<FString> BuildingKeywords = {
        TEXT("building"), TEXT("structure"), TEXT("construction"), 
        TEXT("architecture"), TEXT("house"), TEXT("office")
    };

    for (const FString& Keyword : BuildingKeywords)
    {
        if (TileName.Contains(Keyword, ESearchCase::IgnoreCase))
        {
            return true;
        }
    }

    return false;
}

bool UCesiumSegmentationHandler::IsVegetationTile(const FString& TileName)
{
    // Common patterns for vegetation tiles
    TArray<FString> VegetationKeywords = {
        TEXT("vegetation"), TEXT("tree"), TEXT("forest"), TEXT("plant"), 
        TEXT("grass"), TEXT("foliage"), TEXT("nature")
    };

    for (const FString& Keyword : VegetationKeywords)
    {
        if (TileName.Contains(Keyword, ESearchCase::IgnoreCase))
        {
            return true;
        }
    }

    return false;
}

void UCesiumSegmentationHandler::AddSegmentationRule(const FCesiumTileSegmentationRule& Rule)
{
    Settings.SegmentationRules.Add(Rule);
    UpdateRuleCache();
    
    UE_LOG(LogCesiumSegmentationHandler, Log, 
           TEXT("Added segmentation rule: %s -> ID %d"), 
           *Rule.TileNamePattern, Rule.SegmentationID);
}

bool UCesiumSegmentationHandler::RemoveSegmentationRule(const FString& TileNamePattern)
{
    int32 RemovedCount = Settings.SegmentationRules.RemoveAll(
        [&TileNamePattern](const FCesiumTileSegmentationRule& Rule) {
            return Rule.TileNamePattern.Equals(TileNamePattern, ESearchCase::IgnoreCase);
        }
    );

    if (RemovedCount > 0)
    {
        UpdateRuleCache();
        UE_LOG(LogCesiumSegmentationHandler, Log, 
               TEXT("Removed %d segmentation rules matching pattern: %s"), 
               RemovedCount, *TileNamePattern);
        return true;
    }

    return false;
}

void UCesiumSegmentationHandler::ClearAllSegmentationRules()
{
    Settings.SegmentationRules.Empty();
    UpdateRuleCache();
    
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Cleared all segmentation rules"));
}

void UCesiumSegmentationHandler::UpdateSegmentationSettings(const FCesiumSegmentationSettings& NewSettings)
{
    Settings = NewSettings;
    InitializeAutoIDRange();
    UpdateRuleCache();
    
    // Optionally reprocess all tiles with new settings
    if (Settings.bProcessTilesOnLoad)
    {
        RefreshAllCesiumTileSegmentation();
    }
    
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Updated segmentation settings"));
}

TArray<FString> UCesiumSegmentationHandler::GetProcessedTileNames() const
{
    TArray<FString> Names;
    ProcessedTiles.GetKeys(Names);
    return Names;
}

void UCesiumSegmentationHandler::RefreshAllCesiumTileSegmentation()
{
    // Clear processed tiles cache if caching is disabled
    if (!Settings.bCacheTileData)
    {
        ProcessedTiles.Empty();
    }

    // Reprocess all existing tiles
    ProcessAllExistingTiles();
    
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Refreshed all Cesium tile segmentation"));
}

void UCesiumSegmentationHandler::InitializeAutoIDRange()
{
    NextAutoAssignID = Settings.TileIDRangeStart;
}

bool UCesiumSegmentationHandler::IsAutoIDInRange(uint8 ID) const
{
    return ID >= Settings.TileIDRangeStart && ID <= Settings.TileIDRangeEnd;
}

void UCesiumSegmentationHandler::UpdateRuleCache()
{
    RuleCache.Empty();
    
    for (const FCesiumTileSegmentationRule& Rule : Settings.SegmentationRules)
    {
        if (Rule.bEnabled)
        {
            RuleCache.Add(Rule.TileNamePattern, Rule);
        }
    }
}

void UCesiumSegmentationHandler::LogTileProcessing(const FString& TileName, uint8 AssignedID, const FString& Reason)
{
    UE_LOG(LogCesiumSegmentationHandler, Verbose, 
           TEXT("Processed tile '%s' -> ID %d (%s)"), 
           *TileName, AssignedID, *Reason);
}

void UCesiumSegmentationHandler::LogHandlerState()
{
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("=== Cesium Segmentation Handler State ==="));
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Processed tiles: %d"), ProcessedTiles.Num());
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Tileset actors: %d"), CesiumTilesetActors.Num());
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Active rules: %d"), Settings.SegmentationRules.Num());
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Auto-assign range: %d - %d"), 
           Settings.TileIDRangeStart, Settings.TileIDRangeEnd);
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("Next auto ID: %d"), NextAutoAssignID);
    UE_LOG(LogCesiumSegmentationHandler, Log, TEXT("=========================================="));
}