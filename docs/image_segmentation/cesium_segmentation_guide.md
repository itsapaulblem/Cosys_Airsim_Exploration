# Cesium Terrain Segmentation Guide

This guide explains how to implement image segmentation for Cesium terrain objects in Cosys-AirSim, addressing the common issue where Cesium objects don't appear in segmentation cameras.

## The Problem

Cesium terrain objects don't appear in segmentation images because:

1. **Custom Geometry**: Cesium uses custom, dynamically-streamed geometry that doesn't use standard Unreal `UMeshComponent` types
2. **Procedural Generation**: Terrain meshes are generated at runtime, so there are no persistent actors with standard mesh components at initialization time
3. **Non-Standard Components**: The `FObjectAnnotator` system only recognizes `UStaticMeshComponent` and `USkeletalMeshComponent` objects

## Technical Analysis

### How Segmentation Works
The segmentation system in Cosys-AirSim works by:

1. **Object Discovery**: `FObjectAnnotator::InitializeInstanceSegmentation` finds all `UMeshComponent` instances in the scene
2. **Color Assignment**: Assigns unique colors from a predefined colormap to each mesh component
3. **Material Override**: Creates dynamic material instances (`AnnotationMID`) that override normal materials during segmentation rendering
4. **Rendering**: Special scene proxies render only the annotation materials when capturing segmentation images

### Key Implementation Files
- `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.cpp` - Main annotation logic
- `Unreal/Plugins/AirSim/Source/Annotation/AnnotationComponent.cpp` - Component-level handling
- `Unreal/Plugins/AirSim/Source/PIPCamera.cpp` - Camera implementation
- `Unreal/Plugins/AirSim/Source/RenderRequest.cpp` - Rendering pipeline

## Solutions

### Solution 1: Collision Mesh Workaround (Recommended)

Create simplified collision meshes for Cesium terrain that use standard Unreal components:

#### Step 1: Create Collision Mesh
```cpp
// In your Cesium actor or a separate collision actor
UStaticMeshComponent* CollisionMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("TerrainCollision"));

// Make invisible for normal rendering
CollisionMesh->SetVisibility(false, false);
CollisionMesh->SetHiddenInGame(true);

// But ensure it's processed by annotation system
CollisionMesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
```

#### Step 2: Tag for Segmentation
Add appropriate tags to classify the terrain:

```cpp
// Tag for semantic segmentation
CollisionMesh->ComponentTags.Add(FName("SemanticClasses_1")); // Terrain class
```

#### Step 3: Settings Configuration
```json
{
  "Annotation": [
    {
      "Name": "SemanticClasses",
      "Type": 0,
      "Default": false,
      "SetDirect": false
    }
  ],
  "Vehicles": {
    "Drone1": {
      "Cameras": {
        "segmentation_camera": {
          "ImageType": 10,
          "Annotation": "SemanticClasses",
          "Width": 1024,
          "Height": 768
        }
      }
    }
  }
}
```

### Solution 2: Extend ObjectAnnotator

Modify the annotation system to recognize Cesium components:

#### Step 1: Identify Cesium Components
```cpp
// In ObjectAnnotator.cpp, extend getMeshFromActor function
TArray<UMeshComponent*> FObjectAnnotator::getMeshFromActor(AActor* actor, bool staticMeshOnly) const
{
    TArray<UMeshComponent*> meshes;
    
    // Existing logic for standard components
    TArray<UMeshComponent*> components;
    actor->GetComponents<UMeshComponent>(components);
    
    // Add Cesium-specific handling
    // Look for Cesium3DTileset or other Cesium component types
    TArray<UActorComponent*> cesiumComponents;
    actor->GetComponents(UCesium3DTileset::StaticClass(), cesiumComponents);
    
    for (auto* component : cesiumComponents)
    {
        // Extract renderable geometry from Cesium component
        // This requires understanding Cesium's internal structure
        if (auto* cesiumMesh = ExtractMeshFromCesiumComponent(component))
        {
            meshes.Add(cesiumMesh);
        }
    }
    
    return meshes;
}
```

#### Step 2: Handle Dynamic Geometry
```cpp
// Add callback for when Cesium loads new tiles
void FObjectAnnotator::OnCesiumTileLoaded(UCesium3DTileset* Tileset, const FCesiumTile& Tile)
{
    // Extract mesh components from the loaded tile
    // Apply annotation materials
    if (auto* meshComponent = Tile.GetMeshComponent())
    {
        AddMeshToAnnotation(meshComponent);
    }
}
```

### Solution 3: Custom Annotation Materials

Create custom materials that work with Cesium's rendering pipeline:

#### Step 1: Material Setup
```cpp
// Create a material that can be applied to Cesium geometry
UMaterialInterface* CreateCesiumAnnotationMaterial(FLinearColor SegmentationColor)
{
    UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(
        LoadObject<UMaterial>(nullptr, TEXT("/AirSim/HUDAssets/CesiumAnnotationMaterial")), 
        this
    );
    
    DynamicMaterial->SetVectorParameterValue(FName("SegmentationColor"), SegmentationColor);
    return DynamicMaterial;
}
```

#### Step 2: Apply During Rendering
```cpp
// In render request, override Cesium materials
void FRenderRequest::ApplyCesiumAnnotation()
{
    // Find all Cesium actors
    for (TActorIterator<ACesium3DTileset> ActorItr(World); ActorItr; ++ActorItr)
    {
        ACesium3DTileset* CesiumActor = *ActorItr;
        
        // Override material during segmentation rendering
        if (CesiumActor && CurrentImageType == EImageType::Segmentation)
        {
            OverrideCesiumMaterials(CesiumActor);
        }
    }
}
```

## Configuration Examples

### Basic Terrain Classification
```json
{
  "Annotation": [
    {
      "Name": "TerrainTypes",
      "Type": 0,
      "Default": false,
      "SetDirect": true
    }
  ]
}
```

Tag terrain objects:
- Water: `TerrainTypes_0_0_255` (Blue)
- Grass: `TerrainTypes_0_255_0` (Green) 
- Rock: `TerrainTypes_128_128_128` (Gray)
- Sand: `TerrainTypes_255_255_0` (Yellow)

### Multi-Layer Segmentation
```json
{
  "Annotation": [
    {
      "Name": "TerrainElevation",
      "Type": 1,
      "Default": false
    },
    {
      "Name": "TerrainMaterial",
      "Type": 0,
      "Default": false,
      "SetDirect": false
    }
  ]
}
```

## API Usage

### Python Example
```python
import airsim

client = airsim.MultirotorClient()

# Get terrain segmentation
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Annotation, False, False, "TerrainTypes")
])

# Process the segmentation image
if responses:
    terrain_seg = responses[0]
    # Convert to numpy array and process
    img_array = np.frombuffer(terrain_seg.image_data_uint8, dtype=np.uint8)
    img_array = img_array.reshape(terrain_seg.height, terrain_seg.width, 3)
```

### Blueprint Implementation
1. Create a Blueprint actor that contains collision meshes for terrain
2. Use "Add RGBDirect Annotation Tag to Component" nodes
3. Set appropriate colors for different terrain types
4. Ensure collision mesh is invisible but annotated

## Troubleshooting

### Issue: Terrain Still Not Visible
- Check that collision meshes are properly tagged
- Verify annotation layer is defined in settings.json
- Ensure camera is using correct ImageType and Annotation name

### Issue: Performance Problems
- Use simplified collision meshes, not full-resolution terrain
- Consider LOD systems for distant terrain
- Limit annotation updates to visible areas only

### Issue: Color Conflicts
- Use `simIsValidColor()` to check color availability
- Implement proper color mapping for terrain classification
- Consider using greyscale mode for elevation data

## Best Practices

1. **Simplified Geometry**: Use low-poly collision meshes for large terrain areas
2. **LOD Management**: Implement level-of-detail for annotation meshes
3. **Performance Monitoring**: Watch for frame rate impacts from large annotation areas
4. **Semantic Consistency**: Use consistent color schemes across different terrain types
5. **Documentation**: Keep track of color-to-class mappings for dataset generation

## Future Improvements

- Native Cesium integration in ObjectAnnotator
- Automatic terrain classification based on elevation/slope
- Integration with Cesium's material system
- Support for dynamic terrain streaming