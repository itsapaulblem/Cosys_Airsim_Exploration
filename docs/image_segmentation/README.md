# Image Segmentation in Cosys-AirSim

This directory contains comprehensive documentation about image segmentation capabilities in Cosys-AirSim, including both instance segmentation and the multi-layer annotation system.

## Contents

- **[Instance Segmentation](instance_segmentation.md)** - Automatic unique color assignment to objects for instance segmentation
- **[Annotation System](annotation.md)** - Multi-layer annotation system with RGB, greyscale, and texture modes
- **[Cesium Integration Guide](cesium_segmentation_guide.md)** - Detailed guide for implementing segmentation with Cesium terrain objects
- **[Segmentation Architecture](segmentation_architecture.md)** - Technical details of how segmentation works internally

## Quick Start

### Instance Segmentation
Instance segmentation assigns unique colors to each object automatically:

```json
{
  "SettingsVersion": 2.0,
  "InitialInstanceSegmentation": true,
  "Vehicles": {
    "Drone1": {
      "Cameras": {
        "segmentation_camera": {
          "ImageType": 5,
          "Width": 1024,
          "Height": 768
        }
      }
    }
  }
}
```

### Annotation Layers
Create custom segmentation masks with the annotation system:

```json
{
  "Annotation": [
    {
      "Name": "SemanticClasses",
      "Type": 0,
      "Default": true,
      "SetDirect": false
    }
  ]
}
```

### Cesium Terrain Support
For Cesium terrain objects that don't appear in segmentation:
1. Create collision meshes using standard UStaticMeshComponent
2. Make them invisible for normal rendering but visible for segmentation
3. Tag appropriately for classification

## Key Features

- **Multi-layer Support**: Multiple annotation layers can be used simultaneously
- **RGB, Greyscale, Texture Modes**: Flexible annotation types for different use cases
- **API Integration**: Full Python, C++, and Blueprint API support
- **2.7M+ Unique Colors**: Support for large-scale environments
- **LiDAR Support**: Annotation layers work with GPU LiDAR sensors

## Limitations

- Only Static and Skeletal meshes are supported
- Landscape, Foliage, and Brush objects need workarounds
- Cesium terrain requires special handling
- Maximum 2,744,000 unique colors for instance segmentation

## See Also

- [Image APIs Documentation](../image_apis.md#annotation)
- [GPU LiDAR Documentation](../gpulidar.md)
- [Settings Documentation](../settings.md#subwindows)