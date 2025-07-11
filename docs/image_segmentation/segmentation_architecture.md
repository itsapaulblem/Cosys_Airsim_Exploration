# Segmentation System Architecture

This document provides a technical deep-dive into how the segmentation system works in Cosys-AirSim, covering both instance segmentation and the annotation system.

## Overview

The segmentation system in Cosys-AirSim uses a proxy mesh rendering approach, where objects are rendered with special materials that override their normal appearance to show segmentation information. This approach allows for multiple types of segmentation (instance, semantic, custom annotations) to be generated from the same scene.

## Core Components

### 1. FObjectAnnotator Class

**Location**: `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.cpp`

The central orchestrator of the segmentation system:

```cpp
class FObjectAnnotator
{
private:
    TMap<FString, TSharedPtr<FAnnotator>> Annotators;
    TMap<UMeshComponent*, FLinearColor> ComponentToColor;
    FColorGenerator ColorGenerator;
    
public:
    void InitializeInstanceSegmentation();
    void InitializeAnnotations(const TArray<FAnnotationSettings>& Settings);
    TArray<UMeshComponent*> getMeshFromActor(AActor* actor, bool staticMeshOnly = false) const;
    void SetAnnotationColor(UMeshComponent* Component, const FLinearColor& Color);
};
```

#### Key Responsibilities:
- Discovers all mesh components in the scene at initialization
- Assigns unique colors to objects for instance segmentation
- Manages multiple annotation layers
- Handles runtime updates to object annotations

#### Initialization Process:
1. Iterates through all actors in the world
2. Finds paintable mesh components using `getMeshFromActor()`
3. Creates `UAnnotationComponent` for each mesh
4. Assigns colors from the color generator
5. Sets up material overrides

### 2. UAnnotationComponent Class

**Location**: `Unreal/Plugins/AirSim/Source/Annotation/AnnotationComponent.cpp`

Per-component annotation handling:

```cpp
class UAnnotationComponent : public UActorComponent
{
private:
    UMaterialInstanceDynamic* AnnotationMID;
    FLinearColor AnnotationColor;
    UTexture* AnnotationTexture;
    
public:
    void SetAnnotationColor(const FLinearColor& Color);
    void SetAnnotationTexture(UTexture* Texture);
    void UpdateAnnotationComponent();
    
    // Scene proxy creation for rendering
    virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
};
```

#### Key Features:
- Creates dynamic material instances for annotation rendering
- Manages color and texture override for different annotation modes
- Provides custom scene proxy for segmentation rendering
- Handles both static and skeletal mesh components

### 3. Scene Proxy System

**Custom Scene Proxies**: `FStaticAnnotationSceneProxy`, `FSkeletalAnnotationSceneProxy`

These proxies override the normal rendering pipeline during segmentation capture:

```cpp
class FStaticAnnotationSceneProxy : public FStaticMeshSceneProxy
{
public:
    virtual FMaterialRenderProxy* GetMaterial(int32 ElementIndex) const override
    {
        // Return annotation material instead of normal material
        return AnnotationMaterialProxy;
    }
    
    virtual void GetDynamicMeshElements() const override
    {
        // Render with annotation material
    }
};
```

### 4. Color Generation System

**FColorGenerator Class**:
- Generates up to 2,744,000 unique colors
- Uses deterministic algorithm for consistent color assignment
- Provides mapping between object IDs and RGB colors

```cpp
class FColorGenerator
{
private:
    TArray<FColor> ColorMap;
    int32 CurrentIndex;
    
public:
    FColor GetUniqueColor();
    FColor GetColorByIndex(int32 Index);
    bool IsValidColor(const FColor& Color) const;
};
```

## Rendering Pipeline

### 1. Normal Rendering
```
Scene → Actor → MeshComponent → NormalMaterial → StandardSceneProxy → FinalImage
```

### 2. Segmentation Rendering
```
Scene → Actor → AnnotationComponent → AnnotationMaterial → AnnotationSceneProxy → SegmentationImage
```

### Key Differences:
- **Material Override**: Annotation materials replace normal materials
- **Scene Proxy**: Custom proxies handle annotation-specific rendering
- **Render Target**: Separate render target for segmentation capture
- **Post-Processing**: Disabled during segmentation rendering

## Annotation Types

### 1. Instance Segmentation (ImageType::Segmentation)

**Process**:
1. Automatic color assignment at initialization
2. Each mesh component gets unique color
3. No configuration required
4. Colors generated from deterministic algorithm

**Settings**:
```json
{
  "InitialInstanceSegmentation": true
}
```

### 2. RGB Annotation (Type 0)

**Direct Mode** (`SetDirect: true`):
- Colors specified directly in tags: `LayerName_R_G_B`
- Full control over color assignment
- Useful for semantic segmentation

**Index Mode** (`SetDirect: false`):
- Object IDs mapped to colormap: `LayerName_ID`
- Consistent color assignment
- Automatic color generation

### 3. Greyscale Annotation (Type 1)

**Process**:
- Float values (0.0-1.0) converted to greyscale
- Tag format: `LayerName_Value`
- Useful for continuous data (elevation, distance, etc.)

### 4. Texture Annotation (Type 2)

**Direct Mode**:
- Texture path in tag: `LayerName_/Path/To/Texture`
- Full texture control per object

**Path Reference Mode**:
- Automatic texture loading based on mesh name
- Structured texture organization

## Image Capture Pipeline

### 1. Camera Configuration
```cpp
class APIPCamera : public APawn
{
private:
    USceneCaptureComponent2D* CaptureComponent;
    UTextureRenderTarget2D* RenderTarget;
    
public:
    void SetupSegmentationCapture(const FString& AnnotationName);
    TArray<uint8> CaptureSegmentationImage();
};
```

### 2. Render Request System
```cpp
class FRenderRequest
{
public:
    void SetupAnnotationRendering(const FString& AnnotationName);
    void RestoreNormalRendering();
    TArray<uint8> CaptureImage(EImageType ImageType);
};
```

### 3. Capture Process:
1. **Setup**: Configure scene capture for annotation rendering
2. **Override**: Replace normal materials with annotation materials
3. **Render**: Capture image to render target
4. **Restore**: Return materials to normal state
5. **Extract**: Convert render target to byte array

## Performance Considerations

### Initialization Performance
- **Object Discovery**: O(n) where n = number of actors
- **Component Creation**: O(m) where m = number of mesh components
- **Material Generation**: O(m) dynamic material instances

### Runtime Performance
- **Material Switching**: Minimal overhead during capture
- **Memory Usage**: Additional materials and textures per annotation layer
- **Render Target**: Memory usage based on image resolution

### Optimization Strategies
1. **Lazy Initialization**: Only create annotation components when needed
2. **LOD Integration**: Use appropriate detail levels for annotation
3. **Culling**: Skip annotation for distant or occluded objects
4. **Material Pooling**: Reuse material instances where possible

## API Integration

### C++ API
```cpp
// Instance segmentation
void SetSegmentationObjectID(const std::string& mesh_name, int object_id);
int GetSegmentationObjectID(const std::string& mesh_name);

// Annotation layers
void SetAnnotationObjectColor(const std::string& annotation_name, 
                             const std::string& mesh_name, 
                             int r, int g, int b);
```

### Python API
```python
# Instance segmentation
client.simSetSegmentationObjectID(mesh_name, object_id)
object_id = client.simGetSegmentationObjectID(mesh_name)

# Annotation layers
client.simSetAnnotationObjectColor(annotation_name, mesh_name, r, g, b)
```

### Blueprint API
- **Add RGB Annotation Tag to Component**
- **Update Greyscale Annotation Tag to Actor**
- **Set Texture Annotation by Path**
- **Force Update Annotation Layer**

## Limitations and Workarounds

### Supported Components
✅ **UStaticMeshComponent**: Fully supported
✅ **USkeletalMeshComponent**: Fully supported
❌ **ULandscapeComponent**: Not supported
❌ **UFoliageInstancedStaticMeshComponent**: Not supported
❌ **UBrushComponent**: Not supported

### Workarounds
1. **Landscape**: Convert to static mesh or use collision meshes
2. **Foliage**: Use static mesh instances instead
3. **Procedural Geometry**: Implement custom annotation handling
4. **Third-party Components**: Extend ObjectAnnotator for custom types

## Future Architecture Improvements

### 1. Plugin Architecture
- Modular annotation providers
- Support for third-party segmentation systems
- Runtime plugin loading

### 2. Streaming Support
- Annotation for streaming worlds
- Dynamic object handling
- Memory management for large scenes

### 3. GPU Acceleration
- GPU-based color assignment
- Compute shader material override
- Parallel annotation processing

### 4. Multi-threading
- Background annotation updates
- Asynchronous image capture
- Thread-safe color management

## Debug and Profiling

### Debug Commands
```cpp
// Console commands for debugging
"airsim.annotation.debug 1"           // Enable annotation debug rendering
"airsim.annotation.showcolors 1"      // Display color assignments
"airsim.annotation.stats"             // Show performance statistics
```

### Performance Profiling
- **stat Annotation**: Show annotation system performance
- **stat Rendering**: Monitor render target usage
- **stat Memory**: Track annotation memory usage

### Common Issues
1. **Missing Objects**: Check component type compatibility
2. **Color Conflicts**: Verify color uniqueness
3. **Performance**: Monitor material instance count
4. **Memory Leaks**: Track dynamic material cleanup