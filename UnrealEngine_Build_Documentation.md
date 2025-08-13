# Unreal Engine Build System Documentation

## Table of Contents
1. [Unreal Engine Architecture Overview](#1-unreal-engine-architecture-overview)
2. [Build System Components](#2-build-system-components)
3. [Compilation Process](#3-compilation-process)
4. [Content Pipeline](#4-content-pipeline)
5. [Container Build Workflow](#5-container-build-workflow)
6. [Development Workflow](#6-development-workflow)
7. [Advanced Topics](#7-advanced-topics)

---

## 1. Unreal Engine Architecture Overview

### Core Engine Components and Their Relationships

Unreal Engine is a large and complex C++ project with a modular architecture. The core components can be broadly categorized as follows:

- **Core Systems**: Low-level systems that provide fundamental functionalities like memory management, file I/O, threading, and basic data structures. The `Core` module is the foundation upon which everything else is built.
- **Engine**: This is the heart of Unreal Engine, containing the core gameplay framework, rendering engine, physics system, audio system, and more. It provides the classes and interfaces that developers use to create games and applications.
- **Editor**: The Unreal Editor is a powerful suite of tools for creating and editing content. It is a separate application that is built on top of the Engine. The Editor contains a vast amount of code for its UI, asset management, and various editing modes.
- **Game Projects**: These are the user-created projects, like the "Blocks" project. They contain the game-specific code, assets, and configurations. Game projects are built on top of the Engine and can be run in the Editor or as standalone applications.

The relationship between these components is hierarchical: `Core` is at the bottom, `Engine` builds on `Core`, and `Editor` and `Game Projects` build on the `Engine`.

### Engine/Source Directory Structure

The `Source` directory in an Unreal Engine installation is organized into several key directories:

- `Source/Runtime`: Contains the modules that are required for running a packaged game. This includes the core engine functionalities, rendering, physics, etc. These modules are platform-agnostic.
- `Source/Developer`: Contains modules that are used by developers, but not required in a shipping build. This includes tools for debugging, profiling, and other development-time functionalities.
- `Source/Editor`: Contains the modules for the Unreal Editor. This code is not included in packaged games.
- `Source/ThirdParty`: Contains third-party libraries used by the engine, such as PhysX, Havok, and various platform-specific SDKs.
- `Source/Programs`: Contains source code for various standalone tools that are part of the Unreal Engine ecosystem, such as the Unreal Build Tool and Unreal Automation Tool.

### Key Modules and Their Purposes

Unreal Engine is composed of hundreds of modules. A module is a collection of C++ source code with a `.Build.cs` file that defines how it is built. Here are some of the most important modules:

- **Core**: The absolute base of UE. Contains fundamental types, memory management, string handling, and file system access.
- **CoreUObject**: Implements the UObject system, which provides reflection, garbage collection, and serialization. This is the foundation of the gameplay framework.
- **Engine**: Contains the core gameplay classes (`AActor`, `UWorld`, `AGameModeBase`, etc.), the rendering engine, and other high-level systems.
- **Renderer**: The low-level rendering module. It handles the communication with the graphics API (DirectX, Vulkan, Metal).
- **Slate**: The UI framework for the Unreal Editor and in-game UI.
- **UMG (Unreal Motion Graphics)**: A higher-level UI framework built on top of Slate, designed for creating in-game UI with Blueprints.

### Plugin System Architecture

Plugins are self-contained packages of code and/or content that can be added to an Unreal Engine project. They are a powerful way to extend the engine's functionality without modifying the engine's source code directly.

- **Plugin Structure**: A plugin is a directory containing a `.uplugin` file, which is a JSON file that describes the plugin. It also contains `Source` and `Content` directories for its code and assets.
- **Plugin Types**: Plugins can be of different types, such as `Runtime`, `Editor`, `Developer`, or a combination. This determines which parts of the engine can use the plugin.
- **Loading Phases**: Plugins can be loaded at different times during the engine startup, such as `Default`, `PreDefault`, and `PostConfigInit`. This allows for fine-grained control over when a plugin's code is available.

### Runtime vs. Editor Components

A key concept in Unreal Engine development is the distinction between `Runtime` and `Editor` components.

- **Runtime Components**: These are the parts of the engine and the game project that are needed to run the game as a standalone application. This includes the core engine, gameplay logic, assets, and platform-specific code. The code for these components is typically located in `Source/Runtime` and the project's `Source` directory.
- **Editor Components**: These are the parts of the engine that are only used within the Unreal Editor. This includes the editor's UI, asset editing tools, and other editor-specific functionalities. The code for these components is located in `Source/Editor`.

This separation is enforced by the build system using preprocessor macros like `WITH_EDITOR` and `WITH_EDITORONLY_DATA`. This allows the engine to strip out the editor-specific code and data when packaging a game for distribution, resulting in a smaller and more optimized build.

---

## 2. Build System Components

### Unreal Build Tool (UBT)

The **Unreal Build Tool (UBT)** is a custom C# application that manages the compilation of all C++ code in the engine and game projects.

#### Purpose
UBT is responsible for generating project files for IDEs (like Visual Studio and Xcode), determining which source files need to be compiled, and invoking the appropriate platform-specific compiler and linker.

#### Functionality
- **Module System**: UBT understands the module system of Unreal Engine and can resolve dependencies between modules.
- **Platform Abstraction**: It abstracts the differences between compilers and platforms, allowing for cross-platform compilation from a single source.
- **Build Configuration**: It handles different build configurations (Debug, Development, Shipping, etc.) and sets the appropriate compiler flags and preprocessor definitions.
- **Header Generation**: UBT uses the **Unreal Header Tool (UHT)** to parse C++ headers and generate the necessary code for the UObject reflection system.

#### Workflow
When you build an Unreal Engine project, UBT is invoked. It reads the `.uproject` file, the `.Target.cs` files, and the `.Build.cs` files to understand the project's structure and dependencies. It then generates a build plan and executes it.

### Unreal Automation Tool (UAT)

The **Unreal Automation Tool (UAT)** is another C# application that is used for a wide range of automation tasks, including building, cooking, packaging, and deploying projects.

#### Commands
UAT has a large number of commands for different tasks. Some of the most common ones are:
- `BuildCookRun`: A versatile command that can build, cook, and run a project in a single step.
- `BuildGraph`: A powerful scripting system for creating complex build pipelines.
- `Deploy`: Deploys a build to a device.

#### Capabilities
UAT can be used to automate almost any part of the development and deployment process. It is particularly useful for setting up continuous integration (CI) and continuous delivery (CD) pipelines.

#### Use Cases
- Creating nightly builds of a game.
- Automating the process of cooking and packaging a project for multiple platforms.
- Running automated tests.

### Build Scripts

Your project contains several shell scripts for different platforms:

- **Windows (.bat/.cmd)**: These are batch scripts for automating tasks on Windows. For example, `GenerateProjectFiles.bat` runs UBT to generate Visual Studio project files.
- **Linux (.sh)**: These are shell scripts for automating tasks on Linux. `GenerateProjectFiles.sh` does the same as its `.bat` counterpart, but for a Linux environment (e.g., generating Makefiles or KDevelop/QTCreator project files).

These scripts are typically thin wrappers around UAT or UBT commands.

### Project Files

The build process is configured through a set of C# and JSON files:

#### .uproject
A JSON file that defines the project. It contains information about the project's name, version, and a list of modules and plugins it uses.

#### .uplugin
Similar to `.uproject`, but for plugins. It defines the plugin's modules and other properties.

#### .Build.cs
A C# file that defines a module. It specifies the module's dependencies, include paths, and other build settings.

Example from `Source/Blocks/Blocks.Build.cs`:
```csharp
public class Blocks : ModuleRules
{
    public Blocks(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        
        PublicDependencyModuleNames.AddRange(new string[] { 
            "Core", 
            "CoreUObject", 
            "Engine", 
            "InputCore" 
        });
        
        PrivateDependencyModuleNames.AddRange(new string[] { });
        
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            bEnableExceptions = true;
        }
    }
}
```

#### .Target.cs
A C# file that defines a build target. A target can be a game, an editor, or a server. It specifies which modules to include in the build and the build configuration.

Example from `Source/Blocks.Target.cs`:
```csharp
public class BlocksTarget : TargetRules
{
    public BlocksTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange(new string[] { "Blocks" });
        
        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            bUsePCHFiles = false;
        }
    }
}
```

### Build Configurations

Unreal Engine supports several build configurations:

- **Debug**: Includes full debugging information and disables most optimizations. Useful for debugging low-level engine code.
- **DebugGame**: A hybrid configuration that includes debugging information for the game project, but uses optimized engine code.
- **Development**: The default configuration for development. It includes debugging information and some optimizations.
- **Shipping**: The configuration for the final distributable build. It has full optimizations and no debugging information.
- **Test**: Similar to Shipping, but with some testing and profiling features enabled.

---

## 3. Compilation Process

### C++ Compilation Workflow

The C++ compilation process in Unreal Engine is a multi-stage process orchestrated by the Unreal Build Tool (UBT).

1. **Target Selection**: The process starts with selecting a build target (e.g., `BlocksEditor` for the editor or `Blocks` for the game).
2. **UBT Invocation**: UBT is invoked with the selected target and build configuration (e.g., `Development`, `Shipping`).
3. **Module Discovery**: UBT reads the `.uproject` and `.uplugin` files to discover all the modules that are part of the target.
4. **Dependency Resolution**: UBT reads the `.Build.cs` file for each module to determine its dependencies on other modules. It then creates a dependency graph to determine the correct build order.
5. **Unreal Header Tool (UHT)**: UBT runs the Unreal Header Tool (UHT) on all the C++ header files. UHT parses the headers for `UCLASS`, `USTRUCT`, `UENUM`, `UPROPERTY`, and `UFUNCTION` macros and generates C++ code for the UObject reflection system. This generated code is placed in the `Intermediate` directory.
6. **Compilation**: UBT invokes the platform-specific C++ compiler (e.g., MSVC on Windows, Clang on Linux/macOS) to compile the C++ source files and the UHT-generated files into object files.
7. **Linking**: UBT invokes the linker to link the object files and the required libraries into a final executable or dynamic library.

### Blueprint Compilation

Blueprints are a visual scripting language in Unreal Engine. While they are not C++ code, they are compiled into a more efficient format.

- **Blueprint Nativization**: For shipping builds, Blueprints can be "nativized," which means that they are converted into C++ code and compiled along with the rest of the project. This can significantly improve performance.
- **Just-In-Time (JIT) Compilation**: In the editor and development builds, Blueprints are typically compiled just-in-time (JIT) by a virtual machine.

### Module System

The module system is a key part of the Unreal Engine's architecture. It allows the engine to be broken down into smaller, more manageable pieces.

- **Module Definition**: A module is defined by a `.Build.cs` file. This file specifies the module's name, dependencies, and other build settings.
- **Module Types**: Modules can be of different types, such as `Runtime`, `Editor`, `Developer`, etc. This determines when the module is loaded and what other modules it can depend on.
- **Linking**: Modules can be linked statically or dynamically. By default, most modules are linked dynamically in the editor and statically in packaged builds.

### Dependency Resolution

UBT's dependency resolution is a critical part of the build process.

- **Public vs. Private Dependencies**: In a `.Build.cs` file, you can specify public and private dependencies.
  - `PublicDependencyModuleNames`: Modules that are needed by the public headers of your module.
  - `PrivateDependencyModuleNames`: Modules that are only needed by the private implementation of your module.
- **Circular Dependencies**: UBT will detect and report circular dependencies between modules, which are not allowed.

### Incremental Builds

UBT is designed to optimize rebuild times by only recompiling the files that have changed.

- **Action History**: UBT keeps track of the state of the build in the `Intermediate` directory. It stores information about the timestamps of source files, the command lines used to compile them, and the resulting object files.
- **Change Detection**: When you rebuild a project, UBT checks the action history to see what has changed. If a source file has been modified, it will be recompiled. If a header file has been modified, all the source files that include that header will be recompiled.
- **Precompiled Headers (PCH)**: UBT uses precompiled headers to speed up compilation. A PCH is a precompiled version of a set of commonly used header files. This can significantly reduce the amount of code that needs to be parsed by the compiler.

---

## 4. Content Pipeline

### Cooking Process

"Cooking" is the process of converting Unreal Engine assets from their development format (e.g., `.uasset`, `.umap`) into a platform-specific format that can be loaded by the game.

- **Asset Conversion**: The cooking process converts assets into the optimal format for the target platform. For example, it might compress textures using a platform-specific compression format or convert audio files into a format that is supported by the platform's hardware.
- **Optimization**: The cooking process also optimizes assets for performance. For example, it might generate mipmaps for textures or remove unused data from assets.
- **Cooking on the Fly**: In the editor, assets are cooked "on the fly" when you play the game. This allows for fast iteration times.
- **Commandlet**: The cooking process is performed by a "commandlet," which is a special type of command-line tool that runs within the context of the Unreal Editor. The cooking commandlet can be invoked from the editor's UI or from the command line using UAT.

### Asset Types

Unreal Engine supports a wide variety of asset types, including:

- **Textures**: `.uasset` files containing image data (e.g., PNG, JPG).
- **Models**: `.uasset` files containing 3D models (e.g., FBX, OBJ).
- **Audio**: `.uasset` files containing audio data (e.g., WAV, OGG).
- **Animations**: `.uasset` files containing animation data.
- **Blueprints**: `.uasset` files containing Blueprint graphs.
- **Maps**: `.umap` files containing level data.

### Platform-Specific Cooking

The cooking process is platform-specific. Each platform has its own set of formats and optimizations.

- **Texture Formats**: Different platforms support different texture compression formats. For example, PC uses DXT, while mobile platforms use formats like ASTC or ETC.
- **Shader Compilation**: Shaders are also compiled into a platform-specific format. This is a time-consuming process, so the results are cached in the `DerivedDataCache` directory.

### Packaging

"Packaging" is the process of creating a distributable build of a game. This involves cooking the content, compiling the code, and creating an installer or application bundle.

- **UAT `BuildCookRun`**: The packaging process is typically done using the `BuildCookRun` command in UAT. This command can build the code, cook the content, and package the game in a single step.
- **Output**: The output of the packaging process is a directory containing the game's executable, assets, and other necessary files.

### PAK Files

A PAK file is an archive file that contains a collection of assets.

- **Purpose**: PAK files are used to package assets for distribution. They can improve loading times by reducing the number of individual files that need to be opened. They also make it more difficult for users to modify the game's assets.
- **Structure**: A PAK file has a header that contains a directory of the files it contains. The files are stored in a compressed format.
- **Creation**: PAK files are created during the packaging process. You can choose to package all of your assets into a single PAK file or into multiple PAK files.

---

## 5. Container Build Workflow

### Docker Integration

Using Docker for Unreal Engine builds provides a consistent and reproducible build environment.

- **Dockerfiles**: You can create a Dockerfile that installs all the necessary dependencies for building Unreal Engine, such as the correct version of the C++ compiler, the .NET SDK (for UBT and UAT), and any platform-specific SDKs.
- **Pre-built Images**: There are pre-built Docker images available that contain the Unreal Engine source code and all the necessary dependencies. These can be a good starting point for creating your own custom build images.

### Volume Mounts

When building in a container, you need to mount your project files and build output directories into the container.

- **Project Files**: You should mount your project directory (e.g., `/mnt/d/Cosys_Airsim_Exploration/Unreal/Environments/Blocks`) into the container so that the build tools can access the source code and assets.
- **Build Outputs**: You should also mount a directory for the build outputs (e.g., the `Binaries`, `Intermediate`, and `Saved` directories) so that the build artifacts are persisted outside the container.

### Environment Setup

The container's environment needs to be set up correctly for the build to succeed.

- **Dependencies**: The container needs to have all the necessary dependencies installed, such as the C++ compiler, the .NET SDK, and any platform-specific SDKs.
- **Environment Variables**: You may need to set some environment variables to configure the build process, such as the path to the Unreal Engine source code.

### Cross-Platform Building

Docker can be used to build for different platforms from a single host machine.

- **Linux for Windows**: You can use a Linux container to build for Windows by installing the Windows toolchain in the container.
- **Linux for Linux**: Building for Linux from a Linux container is the most straightforward scenario.

### Container Command Example

The container command you encountered earlier:

```bash
/home/ue4/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh \
    BuildCookRun \
    -utf8output \
    -platform=Linux \
    -clientconfig=Shipping \
    -serverconfig=Shipping \
    -project=/project/Blocks.uproject \
    -noP4 -nodebuginfo -allmaps \
    -cook -build -stage -prereqs -pak -archive \
    -archivedirectory=/project/Packaged
```

This command:
- Uses UAT's `BuildCookRun` command
- Targets Linux platform with Shipping configuration
- Processes all maps in the project
- Performs the complete pipeline: build → cook → stage → pak → archive
- Outputs the final packaged build to `/project/Packaged`

---

## 6. Development Workflow

### Editor Builds

Most of the time, you will be working with an "Editor build" of your project.

- **Building the Editor**: To build the editor for your project, you need to build the `BlocksEditor` target. This will create an editor that is specific to your project and includes all of your project's code and plugins.
- **Running the Editor**: Once the editor is built, you can run it to create and edit your game's content.

### Project Builds

When you want to test your game as a standalone application, you need to create a "project build."

- **Building the Project**: To build your game, you need to build the `Blocks` target. This will create an executable that can be run without the editor.
- **Running the Project**: You can run the project's executable to play your game.

### Plugin Development

If you are developing a plugin, the workflow is similar to developing a game project.

- **Plugin Content**: You can add content to your plugin in the `Content` directory of the plugin.
- **Plugin Code**: You can add C++ code to your plugin in the `Source` directory of the plugin.
- **Building the Plugin**: The plugin will be built along with the project that it is enabled in.

### Version Control Integration

Unreal Engine projects are well-suited for version control systems like Perforce and Git.

- **Perforce**: Perforce is the recommended version control system for Unreal Engine projects, especially for large teams. It has good support for large binary files and has a tight integration with the Unreal Editor.
- **Git**: Git can also be used for Unreal Engine projects, but it requires some configuration to handle large binary files. Git LFS (Large File Storage) is a common solution for this.
- **`.gitignore`**: Your project should have a `.gitignore` file, which is essential for preventing generated files (like the `Intermediate` and `Saved` directories) from being checked into version control.

### CI/CD Integration

The Unreal Automation Tool (UAT) is the key to setting up a continuous integration (CI) and continuous delivery (CD) pipeline for your project.

- **Automated Builds**: You can use UAT to create scripts that automate the process of building, cooking, and packaging your project.
- **Automated Testing**: You can also use UAT to run automated tests to ensure that your project is always in a good state.
- **Jenkins, GitLab CI, etc.**: You can integrate your UAT scripts with CI/CD platforms like Jenkins, GitLab CI, or GitHub Actions to create a fully automated build pipeline.

---

## 7. Advanced Topics

### Custom Build Targets

You can create custom build targets to create specialized builds of your project.

- **Server Targets**: You can create a server target to build a dedicated server for your game.
- **Client Targets**: You can create a client target to build a client that can connect to a dedicated server.
- **Custom Tools**: You can create a custom target to build a standalone tool that uses the Unreal Engine.

To create a custom target, you need to create a new `.Target.cs` file in your project's `Source` directory.

### Build Automation

The Unreal Automation Tool (UAT) provides a powerful scripting system called **BuildGraph** for creating complex build pipelines.

- **BuildGraph Scripts**: BuildGraph scripts are XML files that define a graph of nodes, where each node represents a build task.
- **Use Cases**: BuildGraph can be used to automate a wide range of tasks, such as:
  - Building multiple targets for multiple platforms.
  - Running automated tests.
  - Deploying builds to a server.
  - Generating release notes.

### Performance Optimization

Building a large Unreal Engine project can be time-consuming. Here are some strategies for optimizing build times:

- **Incremental Builds**: Make sure that incremental builds are working correctly. If you are seeing full rebuilds every time, there may be a problem with your project's configuration.
- **Precompiled Headers (PCH)**: Use PCHs to speed up compilation.
- **Unity Builds**: Unity builds (also known as jumbo builds) are a feature of UBT that can speed up compilation by combining multiple C++ files into a single file. This can reduce the overhead of parsing header files.
- **Distributed Builds**: For very large projects, you can use a distributed build system like Incredibuild to distribute the compilation process across multiple machines.

### Debugging Builds

If you are having problems with your build, here are some tools and techniques for debugging them:

- **UBT Log Files**: UBT generates detailed log files that can help you diagnose build problems. The log files are located in the `Intermediate/Build/Logs` directory.
- **Verbose Logging**: You can run UBT with the `-verbose` flag to get more detailed logging information.
- **Debugging UBT**: Since UBT is a C# application, you can attach a debugger to it to step through the code and see what is happening.

### Common Build Commands

#### Windows
```cmd
# Generate project files
GenerateProjectFiles.bat

# Build the project
Engine\Build\BatchFiles\Build.bat BlocksEditor Win64 Development
Engine\Build\BatchFiles\Build.bat Blocks Win64 Shipping

# Package the project
Engine\Build\BatchFiles\RunUAT.bat BuildCookRun -project=Blocks.uproject -platform=Win64 -clientconfig=Shipping -cook -build -stage -prereqs -pak -archive
```

#### Linux
```bash
# Generate project files
./GenerateProjectFiles.sh

# Build the project
./Engine/Build/BatchFiles/Linux/Build.sh BlocksEditor Linux Development
./Engine/Build/BatchFiles/Linux/Build.sh Blocks Linux Shipping

# Package the project
./Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project=Blocks.uproject -platform=Linux -clientconfig=Shipping -cook -build -stage -prereqs -pak -archive
```

---

## Conclusion

This documentation provides a comprehensive overview of the Unreal Engine build system, covering everything from the basic architecture to advanced automation techniques. The modular design of Unreal Engine, combined with the powerful build tools (UBT and UAT), makes it possible to create complex, multi-platform games and applications.

Key takeaways:
- Understanding the module system is crucial for effective Unreal Engine development
- The build system is highly configurable and optimized for incremental builds
- Container-based builds provide consistency and reproducibility
- Automation tools enable sophisticated CI/CD pipelines
- Performance optimization techniques can significantly reduce build times

For further learning, consider exploring the official Unreal Engine documentation and experimenting with the various build configurations and automation scripts in your own projects.