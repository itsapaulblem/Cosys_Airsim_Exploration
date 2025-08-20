#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "Engine/World.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Materials/Material.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "CesiumSegmentationCapture.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogCesiumSegmentation, Log, All);

USTRUCT(BlueprintType)
struct AIRSIM_API FSegmentationObjectData
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    uint8 SegmentationID;

    UPROPERTY(BlueprintReadWrite)
    FLinearColor SegmentationColor; 

    UPROPERTY(BlueprintReadWrite)
    bool bIsValid;

    UPROPERTY(BlueprintReadWrite)
    FString ObjectName;

    FSegmentationObjectData()
    {
        SegmentationID = 0;
        SegmentationColor = FLinearColor::Black;
        bIsValid = false
        ObjectName = TEXT("");
    }

    FSegmentationObjectData(uint8 InSegID, const FString& InName)
    {
        SegmentationID = InSegID;
        SegmentationColor = FLinearColor(InSegID / 255.0f, 0.0f, 0.0f, 1.0f);
        bIsValid = true;
        ObjectName = InName;
    }
};

UCLASS(BlueprintType, Blueprintable)
class AIRSIM_API UCesiumSegmentationCapture : public UObject
{
    GENERATED_BODY()

public:
    UCesiumSegmentationCapture();

    // Initialise the segmentation capture system
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    void Initialize(UWorld* InWorld, const FIntPoint& RenderSize = FIntPoint(1920, 1080));

    // Capture segmentation from a specific viewpoint
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    TArray<FColor> CaptureSegmentation(const FVector& Location, const FRotator& Rotation, float FOV = 90.0f);

    // Set custom segmentation ID for an object
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    bool SetObjectSegmentationID(AActor* Actor, uint8 SegmentationID);


    // Set segmentation ID by object name (supports regex)
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    bool SetObjectSegmentationIDByName(const FString& ObjectName, uint8 SegmentationID, bool bIsRegex = false);

    // Get segmentation ID for an object
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    uint8 GetObjectSegmentationID(AActor* Actor);

    // Get segmentation ID by object name
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    uint8 GetObjectSegmentationIDByName(const FString& ObjectName);

    // Force refresh of all segmentation mappings
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    void RefreshSegmentationMappings();

    // Get all segmentation objects and their data
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    TArray<FSegmentationObjectData> GetAllSegmentationObjects();

    // Get segmentation color map 
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    TMap<FLinearColor> GetSegmentationColorMap();

    // Get list of object names for instance segmentation
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    TArray<FString> GetInstanceSegmentationObjectNames();

    // Validate segmentation assignment
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    bool ValidateSegmentationAssignment(const FString& ObjectName, uint8 ExpectedID);

    // Clear all segmentation assignments
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    void ClearAllSegmentationAssignments();

    // Get redner target for external use
    UFUNCTION(BlueprintCallable, Category = "Segmentation")
    UTextureRenderTarget2D* GetSegmentationRenderTarget() const {
        return SegmentationRenderTarget;
    }

protected:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Segmentation")
    UTextureRenderTarget2D* SegmentationRenderTarget;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Segmentation")
    USceneCaptureComponent2D* SegmentationCapture;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Segmentation")
    UMaterial* SegmentationMasterMaterial;

    UPROPERTY()
    TMap<AActor*, FSegmentationObjectData> ObjectToSegmentationData;

    UPROPERTY()
    TMap<FString, AActor*> ObjectNameToActor; 

    UPROPERTY()
    TMap<uint8, FLinearColor> SegmentationIDToColor;

    UPROPERTY()
    TArray<AActor*> CesiumActors;


    // Settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
    bool bForceRefreshOnCapture;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
    bool bValidateAssignments;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
    bool bDebugVisualisationEnabled;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
    FIntPoint RenderTargetSize; 

    UPROPERTY()
    UWorld* World;

private: 
    void CreateSegmentationRenderTarget();
    void CreateSegmentationCapture();
    bool LoadSegmentationMaterials();
    void UpdateActorSegmentationMaterial(AActor* Actor, uint8 SegmentationID); 
    void ApplySegmentationMaterialToComponent(UMeshComponent* MeshComp, uint8 SegmentationID);
    UMaterialInstanceDynamic* CreateSegmentationMaterialInstance(uint8 SegmentationID);

    // Cesium specific methods
    void IdentifyAndProcessCesiumActors();
    bool IsCesiumActor(AActor* Actor);
    void ProcessCesiumActor(AActor* Actor);

    // Utility methods
    TArray<<AActor*> FindActorsByName(const FString& ObjectName, bool bIsRegex = false);
    bool MatchesRegexPattern(const FString& string, const FString& Pattern);
    void UpdateObjectNameMapping(); 
    void LogSegmentationState();

    // Material cache
    UPROPERTY()
    TMap<uint8, UMaterialInstanceDynamic*> MaterialInstanceCache; 

    void DrawDebugSegmentationInfo();

    // constants
    static const uint8 MAX_SEGMENTATION_ID = 255;
    static const uint8 DEFAULT_TERRAIN_ID = 1;
    static const uint8 DEFAULT_CESIUM_ID = 50;
};
