#include "CesiumSegmentationCapture.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Kismet/KismetRenderingLibrary.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TextureResource.h"
#include "RenderUtils.h"
#include "Regex.h"

DEFINE_LOG_CATEGORY(LogCesiumSegmentation);

UCesiumSegmentationCapture::UCesiumSegmentationCapture()
{
    SegmentationRenderTarget = nullptr;
    SegmentationCapture = nullptr;
    SegmentationMasterMaterial = nllptr;
    World = nullptr; 

    bForceRefreshOnCapture = true;
    bValidateAssignments = true; 
    bDebugVisualisationEnabled = false;
    RenderTargetSize = FIntPoint(1920,1080);
}

bool UCesiumSegmentationCapture::Initialize(UWorld* InWorld, const FIntPoint& RenderSize)
{
    if (!InWorld)
    {
        UE_LOG(LogCesiumSegmentation, Error, TEXT("Invalid world provided for segmentation capture initialization."));
        return false;
    }

    World = InWorld;
    RenderTargetSize = RenderSize;
    
    CreateSegmentationRenderTarget();

    CreateSegmentationCapture();

    // Load materials
    if (!LoadSegmentationMaterials()) 
    {
        UE_LOG(LogCesiumSegmentation, Error, TEXT("Failed to load segmentation materials"));
        return false;
    }

    // Initialise segmentation ID to colour mapping
    for (uint8 i = 0; i < MAX_SEGMENTATION_ID; ++i) {
        SegmentationIDToColor.Add(i, FlinearColor(i / 255.0f, 0.0f, 0.0f, 1.0f));
    }

    // Process existing actors in the world
    RefreshSegmentationMappings();

    UE_LOG(LogCesiumSegmentation, Log, TEXT("Segmentation capture initialized successfully."));
    return true;
}

void UCesiumSegmentationCapture::CreateSegmentationRenderTarget()
{
    SegmentationRenderTarget = UKismetRenderingLibrary::CreateRenderTarget2D(
        World,
        RenderTargetSize.X, 
        RenderTargetSize.Y,
        RTF_RGBA8,
        FLinear::Black,
        false
    );

    if (!SegmentationRenderTarget)
    {
        UE_LOG(LogCesiumSegmentation, Error, TEXT("Failed to create segmentation render target."));
    } else {
        UE_LOG(LogCesiumSegmentation, Log, TEXT("Created segmentation render target: %d x %d"), RenderTargetSize.X, RenderTargetSize.Y);
    }
}

void UCesiumSegmentationCapture::CreateSegmentationCapture()
{
    if (!World)
    {
        return;
    }

    AActor* CaptureActor = World->SpawnActor<AActor>();
    if (!CaptureActor)
    {
        UE_LOG(LogCesiumSegmentation, Error, TEXT("Failed to create capture actor."));
        return;
    }

    SegmentationCapture = Cast<USceneCaptureComponent2D>(
        CaptureActor->AddComponentByClass(USceneCaptureComponent2D::StaticClass(), false, FTransform::Identity, false)
    );

    if (!SegmentationCapture)
    {
        UE_LOG(LogCesiumSegmentation, Error, TEXT("Failed to create scene capture component."));
        return;
    }

    SegmentationCapture->CaptureSource = SCS_FinalColorLDR;
    SegmentationCapture->TextureTarget = SegmentationRenderTarget;
    SegmentationCapture->FOVAngle = 90.0f;
    SegmentationCapture->bCaptureEveryFrame = false; 
    SegmentationCapture->bCaptureOnMovement = false;

    UE_LOG(LogCesiumSegmentation, Log, TEXT("Loaded segmentation master material"));

    return true;
}

TArray<FColor> UCesiumSegmentationCapture::CaptureSegmentation(const FVector& Location, cosnt FRotator& Rotation, float FOV)
{
    TArray<FColor> Result;

    if (!SegmentationCapture || !SegmentationRenderTarget)
    {
        UE_LOG(LogCesiumSegmentation, Error, TEXT("Segmentation capture components not initialised"));
        return Result;
    }

    if (bForceRefreshOnCapture)
    {
        RefreshSegmentationMappings();
    }

    SegmentationCapture->SetWorldLocationAndRotation(Location, Rotation);
    SegmentationCapture->FOVAngle = FOV;

    // Capture the scene
    SegmentationCapture->CaptureScene();

    FTextureRenderTargetResource* RenderTargetResource = SegmentationRenderTarget->GameThread_GetRenderTargetResource();
    if (!RenderTargetResource)
    {
        UE_LOG(LogCesiumSegmentation, Error, TEXT("Failed to get render target resource for segmentation capture."));
        return Result;
    }

    TArray<FColor> SurfaceData;
    if (RenderTargetResource->ReadPixels(SurfaceData))
    {
        Result = SurfaceData;
        UE_LOG(LogCesiumSegmentation, Verbose, TEXT("Captured segmentation image with %d pixels"), Result.Num());
    } else {
        UE_LOG(LogCesiumSegmentation, Error, TEXT("Failed to read pixels from segmentation render target."));
    }

    return Result;
}

bool UCesiumSegmentationCapture::SetObjectSegmentationID(AActor* Actor, uint8 SegmentationID)
{
    if (!Actor)
    {
        UE_LOG(LogCesiumSegmentation, Warning, TEXT("Null actor provided to SetObjectSegmentationID"));
        return false;
    }

    FSegmentationObjectData SegData(SegmentationID, Actor->GetName());
    SegmentationObjectData.Add(Actor, SegData);
    ObjectNameToActor.Add(Actor->GetName(), Actor);

    // Apply the segmentation material 
    UpdateActorSegmentationMaterial(Actor, SegmentationID);

    if (bValidateAssignments)
    {
        bool bSuccess = ValidateSegmentationAssignment(Actor->GetName(), SegmentationID);
        if (!bSuccess)
        {
            UE_LOG(LogCesiumSegmentation, Warning,
                    TEXT("Segmentation assignment validation failed for actor: %s"), *Actor->GetName());
        }

        return bSuccess;
    }

    UE_LOG(LogCesiumSegmentation, Verbose, TEXT("Set segmentation ID %d for actor: %s"), SegmentationID, *Actor->GetName());
    return true;
}

bool UCesiumSegmentationCapture::SetObjectSegmentationIDByName(const FString& ObjectName, uint8 SegmentationID, bool bIsRegex) 
{
    TArray<AActor*> MatchingActors = FindActorsByName(ObjectName, bIsRegex);

    if (MatchingActors.Num() == 0)
    {
        UE_LOG(LogCesiumSegmentation, Warning, TEXT("No actors found mathcing name/pattern: %s"), *ObjectName);
        return false;
    }

    bool bAllSuccessful = true;
    for (AActor* Actor ; MatchingActors)
    {
        bool bSuccess = SetObjectSegmentationID(Actor, SegmentationID);
        if (!bSuccess)
        {
            bAllSuccessful = false;
        }
    }

    UE_LOG(LogCesiumSegmentation, Log, TEXT("Set segmentation ID %d for %d actors matching: %s"), SegmentationID, MatchingActors.Num(), *ObjectName);

    return bAllSuccessful;
}

uint8 UCesiumSegmentationCapture::GetObjectSegmentationID(AActor* Actor)
{
    if (!Actor)
    {
        return 0;
    }

    FSegmentationObjectData* SegData = ObjectToSegmentationData.Find(Actor);
    if (SegData && SegData->bIsValid)
    {
        return SegData->SegmentationID;
    }

    return 0;
}

uint8 UCesiumSegmentationCapture::GetObjectSegmentationIDByName(const FString& ObjectName)
{
    AActor** ActorPtr = ObjectNameToActor.Find(ObjectName);
    if (ActorPtr && *ActorPtr)
    {
        return GetObjectSegmentationID(*ActorPtr);
    }

    return 0;
}

void UCesiumSegmentationCapture::RefreshSegmentationMappings()
{
    if (!World)
    {
        return;
    }

    UE_LOG(LogCesiumSegmentation, Log, TEXT("Refreshing segmentation mappings"));

    UpdateObjectNameMapping();

    IdentifyAndProcessCesiumActors();

    for (auto& Pair : ObjectToSegmentationData)
    {
        if (Pair.Key && Pair.Value.bIsValid)
        {
            UpdateActorSegmentationMaterial(Pair.Key, Pair.Value.SegmentationID);
        }
    }

    if (bDebugVisualizationEnabled)
    {
        LogSegmentationState();
    }

    UE_LOG(LogCesiumSegmentation, Log, TEXT("Segmentation mappings refreshed successfully."));
}

void UCesiumSegmentationCapture::UpdateActorSegmentationMaterial(AActor* Actor, uint8 SegmentationID) 
{
    if (!Actor) {
        return;
    }

    TArray<UMeshComponent*> MeshComponents;
    Actor->GetComponents<UMeshComponent>(MeshComponents);

    for (UMeshComponent* MeshComp : MeshComponents)
    {
        if (MeshComp)
        {
            ApplySegmentationMaterialToComponent(MeshComp, SegmentationID);
        }
    }
}

void UCesiumSegmentationCapture::ApplySegmentionMaterialToComponent(UMeshComponent* MeshComp, uint8 SegmentationID)
{
    if (!MeshComp || !SegmentationMasterMaterial) {
        return;
    }

    for (int32 i = 0; i < MeshComp->GetNumMaterials(); ++i)
    {
        MeshComp->SetMaterial(i, MatInstance);
    }
}

UMaterialInstanceDynamic* UCesiumSegmentationCapture::CreateSegmentationMaterialInstance(uint8 SegmentationID)
{
    // Check cache first
    UMaterialInstanceDynamic** CachedInstance = MaterialInstanceCache.Find(SegmentationID);
    if (CachedInstance && *CachedInstance)
    {
        return *CachedInstance;
    }

    // Create new instance
    UMaterialInstanceDynamic* DynamicMat = UMaterialInstanceDynamic::Create(SegmentationMasterMaterial, this);
    if (!DynamicMat)
    {
        return nullptr;
    }

    // Set segmentation color parameter
    FLinearColor SegColor = FLinearColor(SegmentationID / 255.0f, 0.0f, 0.0f, 1.0f);
    DynamicMat->SetVectorParameterValue(TEXT("SegmentationColor"), SegColor);

    // Cache the instance
    MaterialInstanceCache.Add(SegmentationID, DynamicMat);

    return DynamicMat;
}

void UCesiumSegmentationCapture::IdentifyAndProcessCesiumActors()
{
    if (!World)
    {
        return;
    }

    CesiumActors.Empty();

    // Find all Cesium-related actors
    for (TActorIterator<AActor> ActorIterator(World); ActorIterator; ++ActorIterator)
    {
        AActor* Actor = *ActorIterator;
        if (Actor && IsCesiumActor(Actor))
        {
            CesiumActors.Add(Actor);
            ProcessCesiumActor(Actor);
        }
    }

    UE_LOG(LogCesiumSegmentation, Log, TEXT("Found and processed %d Cesium actors"), CesiumActors.Num());
}

bool UCesiumSegmentationCapture::IsCesiumActor(AActor* Actor)
{
    if (!Actor)
    {
        return false;
    }

    // Check class name for Cesium-related patterns
    FString ClassName = Actor->GetClass()->GetName();
    return ClassName.Contains(TEXT("Cesium")) || 
           ClassName.Contains(TEXT("Tileset")) || 
           ClassName.Contains(TEXT("3DTile"));
}

void UCesiumSegmentationCapture::ProcessCesiumActor(AActor* Actor)
{
    if (!Actor)
    {
        return;
    }

    // If this actor doesn't have a segmentation ID assigned, give it a default one
    if (!ObjectToSegmentationData.Contains(Actor))
    {
        SetObjectSegmentationID(Actor, DEFAULT_CESIUM_ID);
    }
}

TArray<AActor*> UCesiumSegmentationCapture::FindActorsByName(const FString& ObjectName, bool bIsRegex)
{
    TArray<AActor*> Result;
    
    if (!World)
    {
        return Result;
    }

    for (TActorIterator<AActor> ActorIterator(World); ActorIterator; ++ActorIterator)
    {
        AActor* Actor = *ActorIterator;
        if (!Actor)
        {
            continue;
        }

        bool bMatches = false;
        if (bIsRegex)
        {
            bMatches = MatchesRegexPattern(Actor->GetName(), ObjectName);
        }
        else
        {
            bMatches = Actor->GetName().Equals(ObjectName, ESearchCase::IgnoreCase);
        }

        if (bMatches)
        {
            Result.Add(Actor);
        }
    }

    return Result;
}

bool UCesiumSegmentationCapture::MatchesRegexPattern(const FString& String, const FString& Pattern)
{
    const FRegexPattern RegexPattern(Pattern);
    FRegexMatcher RegexMatcher(RegexPattern, String);
    return RegexMatcher.FindNext();
}

void UCesiumSegmentationCapture::UpdateObjectNameMapping()
{
    ObjectNameToActor.Empty();
    
    for (auto& Pair : ObjectToSegmentationData)
    {
        if (Pair.Key)
        {
            ObjectNameToActor.Add(Pair.Key->GetName(), Pair.Key);
        }
    }
}

TArray<FSegmentationObjectData> UCesiumSegmentationCapture::GetAllSegmentationObjects()
{
    TArray<FSegmentationObjectData> Result;
    
    for (auto& Pair : ObjectToSegmentationData)
    {
        if (Pair.Value.bIsValid)
        {
            Result.Add(Pair.Value);
        }
    }

    return Result;
}

TArray<FLinearColor> UCesiumSegmentationCapture::GetSegmentationColorMap()
{
    TArray<FLinearColor> Result;
    
    // Build color map based on current object assignments
    TArray<FSegmentationObjectData> Objects = GetAllSegmentationObjects();
    
    for (const FSegmentationObjectData& ObjData : Objects)
    {
        Result.Add(ObjData.SegmentationColor);
    }

    return Result;
}

TArray<FString> UCesiumSegmentationCapture::GetInstanceSegmentationObjectNames()
{
    TArray<FString> Result;
    
    for (auto& Pair : ObjectToSegmentationData)
    {
        if (Pair.Value.bIsValid)
        {
            Result.Add(Pair.Value.ObjectName);
        }
    }

    return Result;
}

bool UCesiumSegmentationCapture::ValidateSegmentationAssignment(const FString& ObjectName, uint8 ExpectedID)
{
    uint8 ActualID = GetObjectSegmentationIDByName(ObjectName);
    return ActualID == ExpectedID;
}

void UCesiumSegmentationCapture::ClearAllSegmentationAssignments()
{
    ObjectToSegmentationData.Empty();
    ObjectNameToActor.Empty();
    MaterialInstanceCache.Empty();
    
    UE_LOG(LogCesiumSegmentation, Log, TEXT("Cleared all segmentation assignments"));
}

void UCesiumSegmentationCapture::SetDebugVisualizationEnabled(bool bEnabled)
{
    bDebugVisualizationEnabled = bEnabled;
    
    if (bEnabled)
    {
        LogSegmentationState();
    }
}

void UCesiumSegmentationCapture::LogSegmentationState()
{
    UE_LOG(LogCesiumSegmentation, Log, TEXT("=== Segmentation State ==="));
    UE_LOG(LogCesiumSegmentation, Log, TEXT("Total objects: %d"), ObjectToSegmentationData.Num());
    UE_LOG(LogCesiumSegmentation, Log, TEXT("Cesium actors: %d"), CesiumActors.Num());
    UE_LOG(LogCesiumSegmentation, Log, TEXT("Material instances cached: %d"), MaterialInstanceCache.Num());
    
    for (auto& Pair : ObjectToSegmentationData)
    {
        if (Pair.Value.bIsValid)
        {
            UE_LOG(LogCesiumSegmentation, Log, TEXT("  %s -> ID: %d"), 
                   *Pair.Value.ObjectName, Pair.Value.SegmentationID);
        }
    }
    
    UE_LOG(LogCesiumSegmentation, Log, TEXT("========================"));
}
