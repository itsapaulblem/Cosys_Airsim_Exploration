// Fill out your copyright notice in the Description page of Project Settings.
#include "api/WeatherApi.hpp"
#include "WeatherLib.h"
#include "Materials/MaterialParameterCollection.h"
#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"
#include "Blueprint/UserWidget.h"
#include "Blueprint/WidgetBlueprintLibrary.h"
#include "SimMode/SimModeBase.h"
#include "api/ApiProvider.hpp"

AExponentialHeightFog* UWeatherLib::weather_fog_ = nullptr;

UMaterialParameterCollectionInstance* UWeatherLib::getWeatherMaterialCollectionInstance(UWorld* World)
{
    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
    if (World) {
        UMaterialParameterCollection* WeatherParameterCollection = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), NULL, getWeatherParamsObjectPath()));

        //UWorld* World = GetWorld();
        if (WeatherParameterCollection) {
            UMaterialParameterCollectionInstance* Instance = World->GetParameterCollectionInstance(WeatherParameterCollection);
            if (Instance) {
                return Instance;
            }
            else {
                UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get WeatherParameterCollectionInstance1!"));
            }
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get WeatherParameterCollection1!"));
        }
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get World!"));
    }

    return NULL;
}
void UWeatherLib::initWeather(UWorld* World, TArray<AActor*> ActorsToAttachTo)
{
    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
    if (World) {
        UClass* WeatherActorClass = getWeatherActorPath().TryLoadClass<AActor>();
        if (WeatherActorClass) {
            for (int32 i = 0; i < ActorsToAttachTo.Num(); i++) {
                const FVector Location = ActorsToAttachTo[i]->GetActorLocation();
                const FRotator Rotation = ActorsToAttachTo[i]->GetActorRotation();
                FActorSpawnParameters WeatherActorSpawnInfo;
                WeatherActorSpawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                AActor* SpawnedWeatherActor = World->SpawnActor(WeatherActorClass, &Location, &Rotation, WeatherActorSpawnInfo);

                SpawnedWeatherActor->AttachToActor(ActorsToAttachTo[i], FAttachmentTransformRules(EAttachmentRule::SnapToTarget, true));
            }
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid weather actor class!"));
        }
        // still need the menu class for f10
        UClass* MenuActorClass = getWeatherMenuObjectPath().TryLoadClass<AActor>();
        if (MenuActorClass) {
            //UClass* Class, FTransform const* Transform, const FActorSpawnParameters& SpawnParameters = FActorSpawnParameters()
            const FVector Location = FVector(0, 0, 0);
            const FRotator Rotation = FRotator(0.0f, 0.0f, 0.0f);
            FActorSpawnParameters SpawnInfo;
            SpawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            World->SpawnActor(MenuActorClass, &Location, &Rotation, SpawnInfo);
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid menu actor class!"));
        }
    }

    //showWeatherMenu(WorldContextObject);
}
void UWeatherLib::setWeatherParamScalar(UWorld* World, EWeatherParamScalar Param, float Amount)
{
    UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
    if (WeatherMaterialCollectionInstance) {
        FName ParamName = GetWeatherParamScalarName(Param);
        if (ParamName == TEXT("")) {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
        }
        WeatherMaterialCollectionInstance->SetScalarParameterValue(ParamName, Amount);

        // if weather is not enabled, dont allow any weather values to be set
        // must be called after SetScalarParam, because WeatherEnabled is a scalar param
        // and must be set to true or false before this.
        // WeatherEnabled will always be false
        // NOTE: weather enabled must be set first, before other params for this to work
        if (!getIsWeatherEnabled(World)) {
            WeatherMaterialCollectionInstance->SetScalarParameterValue(ParamName, 0.0f);
        }

        // SIMPLE WIND-BASED WEATHER PHYSICS
        float rain = getWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_RAIN);
        float snow = getWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_SNOW);
        float dust = getWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_DUST);
        float fog = getWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_FOG);

        // Calculate wind based on weather intensity
        float total_weather = rain + snow + dust + fog;
        
        if (total_weather > 0.01f) {
            // Generate random wind direction
            float wind_direction = FMath::RandRange(0.0f, 2.0f * PI);
            
            // Calculate wind strength based on weather type and intensity
            float base_wind_speed = 0.0f;
            
            // Different weather types create different wind patterns
            if (rain > 0.01f) {
                base_wind_speed += rain * 25.0f; // Rain creates moderate to strong winds
            }
            if (snow > 0.01f) {
                base_wind_speed += snow * 35.0f; // Snow creates stronger winds
            }
            if (dust > 0.01f) {
                base_wind_speed += dust * 40.0f; // Dust storms have very strong winds
            }
            if (fog > 0.01f) {
                base_wind_speed += fog * 15.0f; // Fog has lighter winds
            }
            
            // Add turbulence variation (±50% variation)
            float turbulence_factor = FMath::RandRange(0.5f, 1.5f);
            float final_wind_speed = base_wind_speed * turbulence_factor;
            
            // Calculate wind components
            float wind_x = final_wind_speed * FMath::Cos(wind_direction);
            float wind_y = final_wind_speed * FMath::Sin(wind_direction);
            float wind_z = final_wind_speed * 0.3f * FMath::RandRange(-1.0f, 1.0f); // Vertical component
            
            // Apply wind using AirSim's existing wind system
            try {
                // Get the SimMode and apply wind through the API
                ASimModeBase* SimMode = ASimModeBase::getSimMode();
                if (SimMode) {
                    // Use the existing wind API that we know works
                    SimMode->getApiProvider()->getWorldSimApi()->setWind(msr::airlib::Vector3r(wind_x, wind_y, wind_z));
                    
                    UE_LOG(LogTemp, Warning, TEXT("Weather Wind Applied: X=%.2f Y=%.2f Z=%.2f (Total Weather=%.2f)"), 
                           wind_x, wind_y, wind_z, total_weather);
                }
            } catch (...) {
                UE_LOG(LogTemp, Error, TEXT("Failed to apply weather wind"));
            }
        } else {
            // No weather - clear the wind
            try {
                ASimModeBase* SimMode = ASimModeBase::getSimMode();
                if (SimMode) {
                    SimMode->getApiProvider()->getWorldSimApi()->setWind(msr::airlib::Vector3r(0, 0, 0));
                    UE_LOG(LogTemp, Log, TEXT("Weather cleared - Wind reset to zero"));
                }
            } catch (...) {
                UE_LOG(LogTemp, Error, TEXT("Failed to clear weather wind"));
            }
        }
        
        // Show fog actor if weather is active
        if (weather_fog_ && total_weather > 0.1f) {
            weather_fog_->GetRootComponent()->SetVisibility(true);
        }
        else if (weather_fog_) {
            weather_fog_->GetRootComponent()->SetVisibility(false);
        }
        
        UE_LOG(LogTemp, Log, TEXT("Weather Visual + Wind Physics Applied: Rain=%.2f, Snow=%.2f, Dust=%.2f, Fog=%.2f"), 
               rain, snow, dust, fog);
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
    }
}
float UWeatherLib::getWeatherParamScalar(UWorld* World, EWeatherParamScalar Param)
{
    UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
    if (WeatherMaterialCollectionInstance) {
        FName ParamName = GetWeatherParamScalarName(Param);
        if (ParamName == TEXT("")) {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
        }
        float Amount;
        WeatherMaterialCollectionInstance->GetScalarParameterValue(ParamName, Amount); //SetScalarParameterValue(ParamName, Amount);

        return Amount;
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
    }
    return 0.0f;
}
FVector UWeatherLib::getWeatherWindDirection(UWorld* World)
{
    UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
    if (WeatherMaterialCollectionInstance) {
        FName ParamName = GetWeatherParamVectorName(EWeatherParamVector::WEATHER_PARAM_VECTOR_WIND);
        if (ParamName == TEXT("")) {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
        }
        FLinearColor Direction;
        WeatherMaterialCollectionInstance->GetVectorParameterValue(ParamName, Direction); //SetScalarParameterValue(ParamName, Amount);

        return FVector(Direction);
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
    }
    return FVector(0, 0, 0);
}
void UWeatherLib::setWeatherWindDirection(UWorld* World, FVector NewWind)
{
    UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
    if (WeatherMaterialCollectionInstance) {
        FName ParamName = GetWeatherParamVectorName(EWeatherParamVector::WEATHER_PARAM_VECTOR_WIND);
        if (ParamName == TEXT("")) {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
        }
        WeatherMaterialCollectionInstance->SetVectorParameterValue(ParamName, NewWind);
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
    }
}
bool UWeatherLib::getIsWeatherEnabled(UWorld* World)
{
    if (getWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_WEATHERENABLED) == 1.0f) {
        return true;
    }
    return false;
}
void UWeatherLib::setWeatherEnabled(UWorld* World, bool bEnabled)
{
    float Value = 0;
    if (bEnabled) {
        Value = 1;
    }
    setWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_WEATHERENABLED, Value);
}
void UWeatherLib::showWeatherMenu(UWorld* World)
{
    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);

    if (UClass* MenuWidgetClass = getWeatherMenuWidgetClass().TryLoadClass<UUserWidget>()) {
        UUserWidget* MenuWidget = CreateWidget<UUserWidget>(World, MenuWidgetClass);

        if (MenuWidget) {
            MenuWidget->AddToViewport();
        }

        APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
        if (PC) {
            PC->bShowMouseCursor = true;
            PC->DisableInput(PC);
        }
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could not load weather widget!"));
    }
}
void UWeatherLib::hideWeatherMenu(UWorld* World)
{

    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
    UClass* MenuWidgetClass = getWeatherMenuWidgetClass().TryLoadClass<UUserWidget>();

    if (World && MenuWidgetClass) {
        // get all menu actors, if any
        TArray<UUserWidget*> FoundWidgets;
        UWidgetBlueprintLibrary::GetAllWidgetsOfClass(World, FoundWidgets, UUserWidget::StaticClass());

        UE_LOG(LogTemp, Warning, TEXT("%s Warning, WeatherAPI"), *MenuWidgetClass->GetClass()->GetFName().ToString());

        if (FoundWidgets.Num() > 0) {
            for (int32 i = 0; i < FoundWidgets.Num(); i++) {
                // hacky test to make sure we are getting the right class. for some reason cast above doesn't work, so we use this instead to test for class
                if (FoundWidgets[i] && FoundWidgets[i]->GetClass()->GetFName().ToString() == getWeatherMenuClassName()) {
                    FoundWidgets[i]->RemoveFromParent();
                }
            }
            APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
            if (PC) {
                PC->bShowMouseCursor = false;
                PC->EnableInput(PC);
            }
        }
    }
}
bool UWeatherLib::isMenuVisible(UWorld* World)
{
    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
    UClass* MenuWidgetClass = getWeatherMenuWidgetClass().TryLoadClass<UUserWidget>();

    if (World && MenuWidgetClass) {
        // get all menu actors, if any
        TArray<UUserWidget*> FoundWidgets;
        UWidgetBlueprintLibrary::GetAllWidgetsOfClass(World, FoundWidgets, UUserWidget::StaticClass());

        UE_LOG(LogTemp, Warning, TEXT("%s Warning, WeatherAPI"), *MenuWidgetClass->GetClass()->GetFName().ToString());

        if (FoundWidgets.Num() > 0) {
            for (int32 i = 0; i < FoundWidgets.Num(); i++) {
                // hacky test to make sure we are getting the right class. for some reason cast above doesn't work, so we use this instead to test for class
                if (FoundWidgets[i] && FoundWidgets[i]->GetClass()->GetFName().ToString() == getWeatherMenuClassName()) {
                    return true;
                }
            }
        }
    }

    // get all menu actors, if any, then hide the menu
    return false;
}
void UWeatherLib::toggleWeatherMenu(UWorld* World)
{
    if (isMenuVisible(World)) {
        hideWeatherMenu(World);
    }
    else {
        showWeatherMenu(World);
    }
}
UWorld* UWeatherLib::widgetGetWorld(UUserWidget* Widget)
{
    if (Widget) {
        return Widget->GetWorld();
    }
    return NULL;
}
UWorld* UWeatherLib::actorGetWorld(AActor* Actor)
{
    if (Actor) {
        return Actor->GetWorld();
    }
    return NULL;
}
void UWeatherLib::setWeatherFog(AExponentialHeightFog* fog)
{
    weather_fog_ = fog;
}