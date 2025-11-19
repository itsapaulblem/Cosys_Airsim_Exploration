#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "Weather/WeatherLib.h"
#include "WeatherPhysicsBridge.generated.h"

// Weather forces structure for Unreal (avoiding AirLib includes)
USTRUCT(BlueprintType)
struct AIRSIM_API FWeatherForces
{
	GENERATED_BODY()
	
	FVector TurbulenceForce = FVector::ZeroVector;
	FVector WindGust = FVector::ZeroVector;
	float DragMultiplier = 1.0f;
	float AirDensityMultiplier = 1.0f;
	bool HasEffects = false;
};

UCLASS(BlueprintType)
class AIRSIM_API UWeatherPhysicsBridge : public UObject
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "Weather Physics")
	static FWeatherForces GetWeatherForces(
		UWorld* World,
		const FVector& Position,
		const FVector& Velocity,
		float DeltaTime
	);

private:
	static float GetWeatherSliderValue(UWorld* World, EWeatherParamScalar Parameter);

	static bool IsWeatherEnabled(UWorld* World);
};	