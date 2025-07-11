# Cesium Segmentation Integration Roadmap

This roadmap outlines the implementation plan for integrating Cesium terrain support into the Cosys-AirSim segmentation system using Solutions 2 and 3 from the [Cesium Segmentation Guide](cesium_segmentation_guide.md).

## Overview

The integration will extend the existing ObjectAnnotator system to recognize Cesium components and implement custom material override systems specifically for Cesium's rendering pipeline.

## Phase 1: Architecture Analysis & Foundation (Week 1-2)

### 1.1 Cesium Component Analysis
- **Research Cesium for Unreal Plugin Architecture**
  - Identify Cesium component types (`UCesium3DTileset`, `UCesium3DTilesetComponent`, etc.)
  - Understand Cesium's rendering pipeline and material system
  - Document Cesium's tile streaming and LOD management
  - Analyze how Cesium handles dynamic geometry updates

### 1.2 Current System Extension Points
- **Extend ObjectAnnotator Discovery System**
  - Modify `getPaintableComponentMeshes()` to support Cesium components
  - Add Cesium-specific component detection logic
  - Create interface for handling non-standard mesh components

### 1.3 Design Cesium Integration Interface
- **Create CesiumAnnotationInterface**
  - Define abstract interface for Cesium component handling
  - Design callback system for tile loading/unloading events
  - Plan material override strategy for Cesium geometry

## Phase 2: Solution 2 - Extend ObjectAnnotator (Week 3-5)

### 2.1 Core ObjectAnnotator Extensions

#### 2.1.1 Component Discovery Enhancement
```cpp
// In ObjectAnnotator.h - Add new methods
void getCesiumPaintableComponents(AActor* actor, TMap<FString, UPrimitiveComponent*>* cesium_components);
bool IsCesiumActor(AActor* actor);
UPrimitiveComponent* ExtractRenderableFromCesium(UActorComponent* cesium_component);
```

#### 2.1.2 Cesium Component Support
- **Extend `getPaintableComponentMeshes()`**
  - Add Cesium component type detection
  - Extract renderable geometry from Cesium tiles
  - Handle dynamic component registration

#### 2.1.3 Runtime Tile Management
- **Implement CesiumTileCallback System**
  - Hook into Cesium tile loading events
  - Automatically register new tiles for annotation
  - Handle tile unloading and cleanup

### 2.2 AnnotationComponent Enhancements

#### 2.2.1 Cesium Scene Proxy Support
```cpp
// New scene proxy class for Cesium components
class FCesiumAnnotationSceneProxy : public FPrimitiveSceneProxy
{
public:
    virtual FMaterialRenderProxy* GetMaterial(int32 ElementIndex) const override;
    virtual void GetDynamicMeshElements() const override;
private:
    UMaterialInterface* AnnotationMaterial;
    FLinearColor SegmentationColor;
};
```

#### 2.2.2 Dynamic Component Handling
- **Support Non-Standard Components**
  - Extend `UAnnotationComponent` for Cesium-specific handling
  - Add support for custom primitive component types
  - Implement dynamic material binding

### 2.3 Integration Points

#### 2.3.1 World Loading Integration
- **Hook into Cesium World Events**
  - Register for tile loading/unloading callbacks
  - Automatic annotation setup for new Cesium actors
  - Handle streaming world scenarios

#### 2.3.2 Settings Integration
```json
// New settings structure in settings.json
"CesiumSegmentation": {
    "EnableAutoDetection": true,
    "TileLoadingTimeout": 5.0,
    "LODThreshold": 2,
    "SupportedTilesetTypes": ["UCesium3DTileset", "UCesiumGlobeAnchor"],
    "MaterialOverrideMode": "Dynamic"
}
```

## Phase 3: Solution 3 - Custom Annotation Materials (Week 6-8)

### 3.1 Cesium Material Override System

#### 3.1.1 Custom Material Infrastructure
```cpp
// New material management class
class FCesiumMaterialOverrideManager
{
public:
    void RegisterCesiumActor(ACesium3DTileset* CesiumActor);
    void ApplyAnnotationMaterial(UPrimitiveComponent* Component, const FLinearColor& Color);
    void RestoreOriginalMaterials(UPrimitiveComponent* Component);
    void HandleTileStreaming(const FCesiumTileLoadEvent& Event);
    
private:
    TMap<UPrimitiveComponent*, UMaterialInterface*> OriginalMaterials;
    UMaterialInstanceDynamic* CreateCesiumAnnotationMaterial(const FLinearColor& Color);
};
```

#### 3.1.2 Render Pipeline Integration
- **Extend RenderRequest for Cesium**
  - Add Cesium-specific material override logic
  - Implement custom rendering path for Cesium components
  - Handle material restoration after segmentation capture

### 3.2 Material Creation System

#### 3.2.1 Dynamic Cesium Materials
- **Create CesiumAnnotationMaterial.uasset**
  - Design material that works with Cesium's rendering pipeline
  - Support for color parameterization
  - Compatible with Cesium's tile streaming
  - Handle different Cesium geometry types

#### 3.2.2 Material Parameter Management
```cpp
// Material parameter handling
class FCesiumAnnotationMaterialParams
{
public:
    void SetSegmentationColor(UMaterialInstanceDynamic* Material, const FLinearColor& Color);
    void SetTextureParameter(UMaterialInstanceDynamic* Material, UTexture* Texture);
    void ConfigureForCesiumGeometry(UMaterialInstanceDynamic* Material);
    void ApplyLODSettings(UMaterialInstanceDynamic* Material, int32 LODLevel);
};
```

### 3.3 Rendering Integration

#### 3.3.1 PIPCamera Extensions
- **Add Cesium Rendering Support**
  - Extend `setCameraTypeEnabled()` for Cesium handling
  - Add Cesium material override during image capture
  - Implement proper cleanup after capture

#### 3.3.2 Scene Capture Integration
- **Cesium-Aware Scene Capture**
  - Override Cesium materials during segmentation capture
  - Handle multiple Cesium actors in single scene
  - Coordinate with tile streaming system

## Phase 4: Testing & Integration (Week 9-10)

### 4.1 Unit Testing Framework
- **Test Component Discovery**
  - Verify Cesium component detection
  - Test with various Cesium actor configurations
  - Validate tile loading/unloading handling

### 4.2 Integration Testing
- **End-to-End Segmentation Testing**
  - Test with real Cesium environments
  - Validate segmentation image quality
  - Performance testing with large tilesets

### 4.3 API Integration Testing
- **Python Client Testing**
  - Test Cesium annotation via API calls
  - Validate color assignment and retrieval
  - Test multi-layer annotation with Cesium

## Phase 5: Documentation & Optimization (Week 11-12)

### 5.1 Performance Optimization
- **Memory Management**
  - Optimize material instance creation
  - Implement material pooling for Cesium
  - Handle large tileset memory usage

### 5.2 Documentation Updates
- **Update Existing Docs**
  - Extend annotation.md with Cesium examples
  - Update settings.md with Cesium configuration
  - Add troubleshooting guide for Cesium issues

### 5.3 Example Implementation
- **Create Cesium Demo Scene**
  - Example Unreal project with Cesium integration
  - Sample settings.json configurations
  - Python scripts demonstrating Cesium annotation

## Technical Implementation Details

### Key Files to Modify:
1. **`ObjectAnnotator.h/.cpp`** - Core component discovery
   - Location: `Unreal/Plugins/AirSim/Source/Annotation/`
   - Changes: Extend component discovery for Cesium types

2. **`AnnotationComponent.h/.cpp`** - Material override system
   - Location: `Unreal/Plugins/AirSim/Source/Annotation/`
   - Changes: Add Cesium scene proxy support

3. **`PIPCamera.h/.cpp`** - Camera integration
   - Location: `Unreal/Plugins/AirSim/Source/`
   - Changes: Cesium-aware rendering pipeline

4. **`RenderRequest.h/.cpp`** - Rendering pipeline
   - Location: `Unreal/Plugins/AirSim/Source/`
   - Changes: Material override coordination

5. **`AirSimSettings.hpp`** - Configuration support
   - Location: `AirLib/include/common/`
   - Changes: Add Cesium segmentation settings

### New Files to Create:
1. **`CesiumAnnotationInterface.h/.cpp`** 
   - Location: `Unreal/Plugins/AirSim/Source/Annotation/`
   - Purpose: Cesium integration interface

2. **`CesiumMaterialOverrideManager.h/.cpp`** 
   - Location: `Unreal/Plugins/AirSim/Source/Annotation/`
   - Purpose: Material management for Cesium

3. **`CesiumAnnotationMaterial.uasset`** 
   - Location: `Unreal/Plugins/AirSim/Content/HUDAssets/`
   - Purpose: Custom material for Cesium

4. **`CesiumSegmentationTests.cpp`** 
   - Location: `AirLibUnitTests/`
   - Purpose: Unit tests for Cesium integration

### Dependencies:
- **Cesium for Unreal Plugin** - Required for Cesium component access
- **Unreal Engine 5.1+** - Minimum version for compatibility
- **Knowledge of Cesium's component architecture** - For proper integration
- **Understanding of Unreal's material system** - For material override implementation

### Risk Mitigation:
- **Cesium API Changes**: Create abstraction layer for Cesium interaction
- **Performance Impact**: Implement lazy loading and material pooling
- **Compatibility**: Maintain backward compatibility with existing annotation system
- **Memory Management**: Proper cleanup of dynamic materials and components

## Implementation Status

### Current Status: Phase 1 - Foundation
- [ ] Cesium component research
- [ ] Interface design
- [ ] Extension point identification

### Next Steps:
1. Research Cesium plugin architecture
2. Design CesiumAnnotationInterface
3. Begin ObjectAnnotator extensions

## Configuration Examples

### Basic Cesium Segmentation Setup
```json
{
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "CesiumSegmentation": {
    "EnableAutoDetection": true,
    "TileLoadingTimeout": 5.0,
    "MaterialOverrideMode": "Dynamic"
  },
  "Annotation": [
    {
      "Name": "CesiumTerrain",
      "Type": 0,
      "Default": false,
      "SetDirect": true
    }
  ],
  "Vehicles": {
    "Drone1": {
      "Cameras": {
        "cesium_segmentation": {
          "ImageType": 10,
          "Annotation": "CesiumTerrain",
          "Width": 1024,
          "Height": 768
        }
      }
    }
  }
}
```

### Python API Usage
```python
import airsim

client = airsim.MultirotorClient()

# Set Cesium terrain color
client.simSetAnnotationObjectColor("CesiumTerrain", "cesium_terrain_tile_1", 0, 255, 0)

# Capture Cesium segmentation
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Annotation, False, False, "CesiumTerrain")
])
```

This roadmap provides a systematic approach to implementing Cesium support in the segmentation system with clear milestones and technical specifications.