#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "Engine/EngineTypes.h" // For Player control.

#include "bot.generated.h"

// @todo: Will need to move them in another file later and using them through namespace.
#define pi		3.1415926535897932384626433832795f
#define tau 	pi*2
#define half_pi	pi/2
#define one_km	100000.0f // Unreal's 1.0 float = 1.0 centimeter

UCLASS()
class A_Bot : public APawn {
	GENERATED_BODY()
	
public:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	class USceneComponent *root;

	// The BoxComponent being used for movement collision.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	class UBoxComponent *collision_box;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	class UCameraComponent *camera;

	static float 	bot_speed;
	static FVector 	objective_vector;

	A_Bot(const FObjectInitializer &ObjectInitializer);
	virtual void PostLoad() override;
	virtual void BeginPlay() override;
	virtual void Tick(float dt_from_tick) override;

	// Inputs are set in project setting in Input category and also you can add and edit inputs in Config->DefaultInput.ini
	virtual void SetupPlayerInputComponent(UInputComponent *input_component) override;
	void reset_ai_logic();

	void simulate_intelligence();
	
	void search_rotation();
	void find_path_point(FVector start_point, FVector start_to_final, float start_to_final_distance, bool found_final_point);
	void set_path_point(FVector start_point, FVector path_point);

	void search_height();
	
	void simulate_input();
	void simulate_rotation();
	void simulate_walking();

	void process_exceptions();
	
	void move_camera();
	void move_bot();
	void raycast();

	// Input logic.
	// Action Mappings:
	void move_forward();
	void move_backward();
	void move_right();
	void move_left();
	void jump();

	void move_forward_released();
	void move_backward_released();
	void move_right_released();
	void move_left_released();
	void jump_released();

	bool is_move_forward_pressed = false;
	bool is_move_backward_pressed = false;
	bool is_move_right_pressed = false;
	bool is_move_left_pressed = false;
	bool is_jump_pressed = false;

	// Axis Mappings:
	void mouse_movement_x(float value);
	void mouse_movement_y(float value);
};
