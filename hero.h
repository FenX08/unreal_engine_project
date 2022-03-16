#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "Engine/EngineTypes.h" // For Player control.

#include "hero.generated.h"

// @todo: Will need to move them in another file later and using them through namespace.
#define pi					3.1415926535897932384626433832795f
#define tau 				pi*2
#define half_pi				pi/2
//#define infinity_positive	3.4E+38
#define infinity_positive	TNumericLimits<float>::Max()
//#define infinity_negative	1.2E-38
#define infinity_negative	TNumericLimits<float>::Min()
#define one_km				100000.0f // Unreal's 1.0 float = 1.0 centimeter

// @todo: I can avoid inhereting from APawn class, if I add my own PlayerInputComponent.
UCLASS()
class A_Player : public APawn {
	GENERATED_BODY()
	
public:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	class USceneComponent *root;

	// The BoxComponent being used for movement collision.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	class UBoxComponent *collision_box;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	class UCameraComponent *camera;

	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Player")
	//int32 value;

	static bool 	game_started;
	static float 	player_speed;
	static FVector 	player_position;
	static AActor 	*player;

	// Execution of the entity comes in this order:
	// Class() -> PostLoad() -> BeginPlay() -> Tick()
	A_Player(const FObjectInitializer &ObjectInitializer);
	virtual void PostLoad() override;
	virtual void BeginPlay() override;
			void spawn_additional_entities_for_player();
	virtual void Tick(float dt) override;
			void send_variables_to_post_update();


	// Inputs are set in project setting in Input category and also you can add and edit inputs in Config->DefaultInput.ini
	virtual void SetupPlayerInputComponent(UInputComponent *input_component) override;

	void move_camera(float dt);
	void move_player(float dt);
	void raycast(float dt);
	void time_control(float dt);

	// Input logic.
	// Action Mappings:
	void move_forward();
	void move_backward();
	void move_right();
	void move_left();
	void jump();
	void time_rewind();

	void move_forward_released();
	void move_backward_released();
	void move_right_released();
	void move_left_released();
	void jump_released();
	void time_rewind_released();

	bool is_move_forward_pressed = false;
	bool is_move_backward_pressed = false;
	bool is_move_right_pressed = false;
	bool is_move_left_pressed = false;
	bool is_jump_pressed = false;
	bool is_time_rewind_pressed = false;

	// Axis Mappings:
	void mouse_movement_x(float value);
	void mouse_movement_y(float value);
};
