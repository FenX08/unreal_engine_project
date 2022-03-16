#include "hero.h"
#include "hui.h"
#include "post_update.h"

#include "Components/SceneComponent.h"
#include "Components/BoxComponent.h" // For collision.
#include "Camera/CameraComponent.h"
#include "DrawDebugHelpers.h"

#include "cd_core/log.h"

bool 	A_Player::game_started 		= false;
float 	A_Player::player_speed 		= 0;
FVector A_Player::player_position 	= FVector(0);
AActor 	*A_Player::player;

namespace {
	float	collision_size 		= 20.0f;
	// Collizion Z is actually twice this number. It's height is a sum of collision_height up and down.
	// So if collision_height = 80.0f, whole collision height will be 160.0f in Unreal.
	float	collision_height 	= 92.0f;
	FVector collision_bounds(collision_size, collision_size, collision_height);
	FVector collision_velocity;
	
	// Camera move variables.
	FBodyInstance 	*collision_physics;
	FVector 		camera_offset(0.0f, 0.0f, 70.0f);
	FVector 		camera_forward_vector;
	FVector 		camera_backward_vector;
	FVector 		camera_right_vector;
	FVector 		camera_left_vector;
	FVector 		camera_up_vector;
	FVector 		camera_down_vector;
	
	float 						mouse_input_x;
	float 						mouse_input_y;
	float 						mouse_design_sensitivity 		= 100.0f;
	float 						mouse_user_overall_sensitivity 	= 1.0f;
	float 						mouse_user_sensitivity_x		= 1.0f;
	float 						mouse_user_sensitivity_y		= 1.0f;
	float 						mouse_camera_smoothness			= 1.0f; // @note: I don't use camera smoothing. Works without delta time, 1.0 means no smoothing.
	FRotator 					camera_euler_rotation(0);
	FRotationConversionCache 	camera_rotation_conversion;	// @note: This is for optimization, I probably don't use it fully.
	
	// Player move variables.
	FRotationConversionCache 	collision_rotation_conversion; // @note: This is for optimization, I probably don't use it fully.
	
	// @todo: rename these to speed.
	float			max_walking_speed		= 2400.0f;
	float 			forward_force 			= 2400.0f;
	float 			backward_force 			= 2400.0f;
	float 			right_force 			= 2400.0f;
	float 			left_force 				= 2400.0f;
	float 			jump_force 				= 3700.0f; 	// Default gravity Z is -980.0
	float 			gravity_extra_force 	= 1077.0f; 	// I make gravity stronger by adding extra force.
	
	float 			drag_walking_force		= 1.3f; // Drag walking should be less than stopping.
	float 			drag_stop_walking_force = 3.2f;
	
	bool 			is_walking = false;	// Right now I control this by key input and not by checking player velocity.
	
	bool 			mass_has_no_effect = true;
	
	// @note: Hardcoded vectors for move input keys on unit circle.
	FVector			move_forward_xy		(0,1,0);
	FVector			move_backward_xy	(0,-1,0);
	FVector			move_right_xy		(1,0,0);
	FVector			move_left_xy		(-1,0,0);
	
	struct Player_State {
		FVector position = FVector(0);
		FQuat 	rotation = FQuat::Identity;
		FVector velocity = FVector(0);
	} player_state;
	
	struct World_State {
		Player_State saved_player = player_state;
	} world_state;
	
	// In array the size will be memory_size * World_State, so be carefull
	// when you add new types to states.
	// 20 million bytes * 48 is around 1 GB.
	const int world_array_memory_size 	= 20000000;
	struct State_Array {
		World_State world[world_array_memory_size];
		int64 		world_count 		= 0;
		int64 		world_memory_size 	= 0;
	} state_array;
	
	int64 world_state_type_memory_size 	= sizeof(World_State);
	
	float save_world_state_timer 	= 0;
	float rewinding_timer 			= 0;
	bool allowed_to_rewind 			= false;
	bool currently_rewinding 		= false;
}

A_Player::A_Player(const FObjectInitializer &ObjectInitializer) : Super(ObjectInitializer) {
	root = ObjectInitializer.CreateDefaultSubobject<USceneComponent>(this, TEXT("Root Component"));
	SetRootComponent(root);
	
	player = this;

	// Player Inputs are procceced in TG_PrePhysics,
	// so we are ensuring that our player update executes as soon as possible.
	// We can also give different TickGroup for different player components.
	PrimaryActorTick.bCanEverTick 			= true;
	PrimaryActorTick.bTickEvenWhenPaused 	= true;
	PrimaryActorTick.TickGroup 				= TG_PrePhysics;
	
	collision_box = ObjectInitializer.CreateDefaultSubobject<UBoxComponent>(this, TEXT("Collision Box"));
	collision_box->SetupAttachment(root);
	
	bool is_registered_and_collides = true;
	collision_box->SetBoxExtent(collision_bounds, is_registered_and_collides);
	collision_box->SetSimulatePhysics(true);
	
	// Get adress of a struct that we can use for physical manipulations. 
	collision_physics = collision_box->GetBodyInstance();
	
	// We will lock X and Y rotation of collision, so that our collision
	// box wouldn't fall on the ground.
	collision_physics->bLockYRotation = 1;
	collision_physics->bLockXRotation = 1;
	
	collision_physics->LinearDamping = 0; // I use my own drag forces.

	// @hack: I use this so that my collision wouldn't rotate after given it a walking impulse.
	collision_physics->InertiaTensorScale = FVector(10000.0f);

	collision_box->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	collision_box->SetShouldUpdatePhysicsVolume(true);
	//collision_box->bDynamicObstacle = true;
	//collision_box->SetCanEverAffectNavigation(false);
	//collision_box->CanCharacterStepUpOn = ECB_No;
	
	camera = ObjectInitializer.CreateDefaultSubobject<UCameraComponent>(this, TEXT("Camera"));
	camera->SetupAttachment(root);
	
	camera->bAutoActivate 			= true;
	camera->FieldOfView 			= 104.0f;
	camera->bUsePawnControlRotation = false;
	
	// Offset camera in box collision by desired vector.
	camera->SetRelativeLocation(camera_offset);
}

void A_Player::PostLoad() {
	Super::PostLoad();
	
	// We give Player control on this entity.
	AutoReceiveInput = EAutoReceiveInput::Player0;
}

void A_Player::SetupPlayerInputComponent(UInputComponent *input_component) {
	Super::SetupPlayerInputComponent(input_component);
	
	// @note: IE_Repeat parameter works slow (with delay). Something to do with text typing?
	input_component->BindAction("move_forward", IE_Pressed, this, &A_Player::move_forward);
	input_component->BindAction("move_backward", IE_Pressed, this, &A_Player::move_backward);
	input_component->BindAction("move_right", IE_Pressed, this, &A_Player::move_right);
	input_component->BindAction("move_left", IE_Pressed, this, &A_Player::move_left);
	input_component->BindAction("jump", IE_Pressed, this, &A_Player::jump);
	input_component->BindAction("time_rewind", IE_Pressed, this, &A_Player::time_rewind);
	
	input_component->BindAction("move_forward", IE_Released, this, &A_Player::move_forward_released);
	input_component->BindAction("move_backward", IE_Released, this, &A_Player::move_backward_released);
	input_component->BindAction("move_right", IE_Released, this, &A_Player::move_right_released);
	input_component->BindAction("move_left", IE_Released, this, &A_Player::move_left_released);
	input_component->BindAction("jump", IE_Released, this, &A_Player::jump_released);
	input_component->BindAction("time_rewind", IE_Released, this, &A_Player::time_rewind_released);
	
	input_component->BindAxis("mouse_movement_x", this, &A_Player::mouse_movement_x);
	input_component->BindAxis("mouse_movement_y", this, &A_Player::mouse_movement_y);
}

void A_Player::BeginPlay() {
	Super::BeginPlay();

	spawn_additional_entities_for_player();
	
	// I use it to reset desired global variables when player spawned in singleplayer.
	// @note: You can use PostLogin (Handle Starting New Player blueprint) and OnLogout in GameMode for controlling game starting and ending.
	// Right now it's always true. Are you okay with that?
	game_started = true;

	// Interestingly, if I begin playing, this global variable doesn't reset, if I'm restarting the map.
	// So I'm resetting this camera vector cache for now.
	camera_euler_rotation = FRotator(0);

	// Reset state arrays before new playing new game.
	state_array.world_count = 0;
	state_array.world_memory_size = 0;
	save_world_state_timer = 0;
}

void A_Player::spawn_additional_entities_for_player() {	
	// Example on spawning an entity.
	//FActorSpawnParameters post_update_spawn_info;
	//AActor *post_update = GWorld->SpawnActor<A_Post_Update>(A_Post_Update::StaticClass(), FVector(0), FRotator(0), post_update_spawn_info);
	
	// Spawn post_update entity that will execute post physics code for player.
	AActor *post_update = GWorld->SpawnActor<A_Post_Update>(A_Post_Update::StaticClass());
}

void A_Player::Tick(float dt) {
	Super::Tick(dt);

	// @todo: Make dt global.

	// @note: If player entity gets too high (or too low?) Unreal will delete it.
	// @bug: If you pause in editor play, while falling, and you wait a little, velocity becomes very high and collision passes through ground after you press play and continue.
	player_position = GetActorLocation();
	//UE_LOG(Log_CD_Core, Log, TEXT("Player position: %s"), *GetActorLocation().ToString());
	
	move_camera(dt);

	if (!currently_rewinding) {
		move_player(dt);
	}

	// @todo: What's the execution order of physics? By now, raycast will be always behind one frame.
	// This is okay for now, but in the future we will need current frame precision.
	// Is this even possible with Unreal Engine's execution order?
	//raycast(dt);

	time_control(dt);

	send_variables_to_post_update();
}

void A_Player::send_variables_to_post_update() {
	// @hack: Separate entity for post update, after all physics are done.
	// I could have done it, by making separate tick functions for player, but I'm lazy to work through this Unreal Engine tick logic.
	// @bug: Game crushes if player was deleted.
	A_Post_Update::player_root_entity = root;
	A_Post_Update::player_collision_box_entity = collision_box;
}

void A_Player::move_camera(float dt) {
	// @speed: Right now I use euler rotation for mouse input and convert euler to quternion.
	// I need to learn how to use quaternion only for rotation inputs. If I do, this code will become faster.
	
	// @note: On low FPS (like 30) camera moves faster. Probably something to do with parallel input
	// processing in Unreal Engine or maybe that it's only 30 cycles in game frame timer, don't know.
	// Maybe later I could change camera move speed depending on current framerate.
	FRotator camera_mouse_rotation(0);
	camera_mouse_rotation.Yaw 	= mouse_input_x * mouse_design_sensitivity * mouse_user_overall_sensitivity * mouse_user_sensitivity_x * dt;
	camera_mouse_rotation.Pitch = mouse_input_y * mouse_design_sensitivity * mouse_user_overall_sensitivity * mouse_user_sensitivity_y * dt;
	
	camera_euler_rotation += camera_mouse_rotation;
	camera_euler_rotation.Roll = 0.0f; // @note: Potentially can be used for head leaning, but I will lock this axis for now.
	
	// Clamp Pitch value to move camera not more than straight down and up.
	camera_euler_rotation.Pitch = FMath::Clamp(camera_euler_rotation.Pitch, -90.f, 90.0f);
	
	// Rotate camera without smoothing. This is Normilized Lerp method.
	// Formula: q = lerp(q1, q2, 0.1).normalize()
/*
		FQuat old_quaternion_rotation = camera->GetComponentQuat();
		FQuat target_quaternion_rotation = FQuat(FVector::UpVector, FMath::DegreesToRadians(camera_euler_rotation.Yaw)) * FQuat(FVector::LeftVector, FMath::DegreesToRadians(camera_euler_rotation.Pitch));
		FQuat new_quaternion_rotation = FQuat::FastLerp(old_quaternion_rotation, target_quaternion_rotation, mouse_camera_smoothness);
		new_quaternion_rotation.Normalize();
*/
	
	// Rotate camera without smoothing.
	FQuat new_quaternion_rotation = camera_rotation_conversion.RotatorToQuat(camera_euler_rotation);

	// Set new rotation.
	camera->SetRelativeRotation(new_quaternion_rotation);
}

void A_Player::move_player(float dt) {
	// We rotate box collision by Z axes with the camera.
	FRotator collision_box_rotation = collision_box->GetRelativeRotation();
	collision_box_rotation.Yaw 		= camera_euler_rotation.Yaw;
	FQuat new_collision_rotation 	= collision_rotation_conversion.RotatorToQuat(collision_box_rotation);
	collision_box->SetRelativeRotation(new_collision_rotation);
	
	// Get direction vectors of collision for movement.
	FVector collision_forward_vector 	= collision_box->GetForwardVector();
	FVector collision_up_vector 		= collision_box->GetUpVector();

	// If we are not pressing any walking buttons, we are not walking.
	if (!is_move_forward_pressed && !is_move_backward_pressed && !is_move_right_pressed && !is_move_left_pressed) {
		is_walking = false;
	}

	if (is_walking) {
		FVector current_walking_vector(0, 0, 0);
		float current_speed = 0;

		// @note: Right now I do not use gamepad stick XY inputs.
		if (is_move_forward_pressed) {
			current_walking_vector += move_forward_xy;
			current_speed += forward_force;
		}
		
		if (is_move_backward_pressed) {
			current_walking_vector += move_backward_xy;
			current_speed += backward_force;
		}
		
		if (is_move_right_pressed){
			current_walking_vector += move_right_xy;
			current_speed += right_force;
		}
		
		if (is_move_left_pressed) {
			current_walking_vector += move_left_xy;
			current_speed += left_force;
		}

		// Normalize walking vector to fit into unit circle.
		current_walking_vector.Normalize();
		
		// We find angle through dot product and we use found number in arccosine (or inverse cosine) to get angle in radians,
		// between our vector and (as I call it) sine vector.
		// If our vector is poiting at negative X axis, we need compensate that this angle to get past 180 degrees.
		// I do that to player collision so that it perfectly translates from Unreal XY bottom-up view to my XY top-down unit circle.
		float collision_angle = FMath::Acos(FVector::DotProduct(collision_forward_vector, FVector(0,1,0)));
		if (collision_forward_vector.X < 0) {
			collision_angle = tau - collision_angle;
		}
		
		// For inputs we are comparing to cosine vector.
		// If our vector is poiting at negative Y axis, then we turned past pi, and we need to subtract found angle from full circle to get correct angle.
		// We do this for correcting XY input interpretation to place it on unit circle from 0 degrees (x=1,y=0) to 360 degrees (x=1,y=0).
		float input_angle = FMath::Acos(FVector::DotProduct(current_walking_vector, FVector(1,0,0)));
		if (current_walking_vector.Y < 0) {
			input_angle = tau - input_angle;
		}

		// Correct collision angle. It's like turning trig circle on 90 degrees to the left.
		// Think about it as if input angle has sine relationship and collision angle has cosine relationship,
		// and final angle for world space would have sine relationship.
		// I found this pattern by investigating unit circle; I think I understand it,
		// but for now I can't reason this idea quite well into words.
		// -- Richard Chirkin 12.01.2022
		collision_angle -= half_pi;

		// We add corrected player collision angle to input angle on unit circle and we get correct angle for player walking direction.
		float direction_angle = collision_angle + input_angle;
		
		// Because Unreal Engine XY axis are placed underneath Z axis (look at XY from bottom-up view),
		// we need to translate direction angle from unit circle (XY top-down) to Unreal YX top-down view (right is Y = 1 and forward is X = 1).
		direction_angle = tau - (direction_angle - half_pi);

		//UE_LOG(Log_CD_Core, Log, TEXT("angle: %.2f"), direction_angle * (180/pi));
		
		// Finally, we find the right normalized XY coordinates from cosine and sine angle. 
		FVector direction_vector(0,0,0);
		FMath::SinCos(&direction_vector.Y, &direction_vector.X, direction_angle);

		// If we got several directions - clamp speed;
		// @note: 	We can use input_angle to find the difference between forward and right speed, etc.,
		// 			for example like adding to perpendicular speeds (forward and left) if they are different.
		// 			Formula is something like: (right_speed / 2) + (forward_speed / 2). Should be useful for stick input.
		if (current_speed > max_walking_speed) {
			current_speed = max_walking_speed;
		}

		// Tell physics system in what direction should we move with our constracted walking direction vector.
		collision_physics->AddImpulse(direction_vector * (current_speed * dt), mass_has_no_effect);
	}

	if (is_jump_pressed) {
		collision_physics->AddImpulse(collision_up_vector * (jump_force * dt), mass_has_no_effect);
	}
	
	// Add force to gravity.
	collision_physics->AddImpulse(-collision_up_vector * (gravity_extra_force * dt), mass_has_no_effect);
	
	// We are clamping our walking velocity through drag forces.
	// If were are not walking, we are initiating more powerfull drag force.

	// @note: You can set instant velocity to collision with SetPhysicsLinearVelocity in PrimitiveComponent.

	// @todo: This drag method will also try to stop you if you are not pressing any buttons and in air.
	// I need moving states for this one, because I also moving very fast while falling.
	// For example, if I move in air I should constraint magnitude of move_input vectors.

	collision_velocity = collision_physics->GetUnrealWorldVelocity();

	// Debug velocity.
	//UE_LOG(Log_CD_Core, Log, TEXT("Velocity: %s"), *collision_velocity.ToString());

	collision_velocity.Z = 0; // We do not use gravity, this is only for XY axes.
	
	// I'm taking current player velocity, inverse it and multiply it by desired drag force.
	if (!is_walking) {
		collision_physics->AddImpulse(-collision_velocity * (drag_stop_walking_force * dt), mass_has_no_effect);
	} else {
		collision_physics->AddImpulse(-collision_velocity * (drag_walking_force * dt), mass_has_no_effect);
	}

	// We can know player speed by finding magnitude (length, e.g. speed) in velocity vector.
	// This speed includes Z height velocity.
	collision_velocity = collision_physics->GetUnrealWorldVelocity();
	player_speed = collision_velocity.Size();
}

void A_Player::raycast(float dt) {
	FHitResult 				out_hit;
	FCollisionQueryParams 	collision_parameters;
	
	// Ignore player collision.
	collision_parameters.AddIgnoredActor(this);

	FVector forward_vector 	= collision_box->GetForwardVector();
	FVector down_vector 	= -collision_box->GetUpVector();
	FVector start 			= collision_box->GetComponentLocation();
	FVector end 			= start + (down_vector * one_km);
	
	bool got_hit = GetWorld()->LineTraceSingleByChannel(out_hit, start, end, ECC_Visibility, collision_parameters);
	
	if (got_hit) {
		if (out_hit.bBlockingHit) {
			FVector point_hit 			= out_hit.ImpactPoint;
			FVector point_hit_normal 	= out_hit.ImpactNormal;
			
			// Draw vector of the first collision that we hit.
			DrawDebugLine(GetWorld(), start, point_hit, FColor::Black, false, dt + 0.0001f, 0, 1.2f);
			
			//UE_LOG(Log_CD_Core, Log, TEXT("Vector Z of hit: %.2f"), point_hit.Z);

			// Drawn normal of found point on collision.
			//DrawDebugLine(GetWorld(), point_hit, point_hit + point_hit_normal * 500.0f, FColor::Red, false, dt + 0.0001f, 0, 1.2f);
			
			if (GEngine) {
				//GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::White, FString::Printf(TEXT("You're hitting: %s"), *out_hit.GetActor()->GetName()));
				//GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::White, FString::Printf(TEXT("Impact point: %s"), *out_hit.ImpactPoint.ToString()));
				//GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::White, FString::Printf(TEXT("Normal point: %s"), *out_hit.ImpactNormal.ToString()));
			}
		}
	} else {
		// If no hit, trace a line with a different color.
		DrawDebugLine(GetWorld(), start, end, FColor::Red, false, dt + 0.0001f, 0, 1.2f);
	}
}

void A_Player::time_control(float dt) {
	// If you rewind the time, what will you do with Unreal clouds and sun?

	// FTransform 	= 46 byte
	// FVector 		= 12 byte
	// FQuat 		= 16 byte

	save_world_state_timer += dt;
	rewinding_timer += dt;
	
	// Shouldn't we do this post physics?
	if (
		//save_world_state_timer >= 1 &&
		!currently_rewinding) {
		player_state.position = collision_box->GetComponentLocation();
		player_state.rotation = collision_box->GetComponentQuat();
		player_state.velocity = collision_velocity;

		world_state.saved_player = player_state;

		state_array.world[state_array.world_count] = world_state;
		++state_array.world_count;
		state_array.world_memory_size += world_state_type_memory_size;

		save_world_state_timer = 0;

		allowed_to_rewind = true;

		/*
		UE_LOG(Log_CD_Core, Log, TEXT("SAVE ============================"));
		UE_LOG(Log_CD_Core, Log, TEXT("player_state.position: %s"), *player_state.position.ToString());
		UE_LOG(Log_CD_Core, Log, TEXT("player_state.rotation: %s"), *player_state.rotation.ToString());
		UE_LOG(Log_CD_Core, Log, TEXT("player_state.velocity: %s"), *player_state.velocity.ToString());
		UE_LOG(Log_CD_Core, Log, TEXT("state_array.world_count: %d"), state_array.world_count);
		UE_LOG(Log_CD_Core, Log, TEXT("World memory: %d\t||\tRemaining memory: %d"), state_array.world_memory_size, world_array_memory_size);
		*/
	}

	// @todo: We need to disable colliding with static collision. But we could stuck in collision,
	// if we realease rewind at the right moment.
	if (
		//rewinding_timer >= 1 &&
		is_time_rewind_pressed && allowed_to_rewind) {
		currently_rewinding = true;
		rewinding_timer = 0;

		FVector player_saved_position = state_array.world[state_array.world_count].saved_player.position;
		FQuat 	player_saved_rotation = state_array.world[state_array.world_count].saved_player.rotation;
		FVector player_saved_velocity = state_array.world[state_array.world_count].saved_player.velocity;

		collision_box->SetRelativeLocation(player_saved_position);
		collision_box->SetRelativeRotation(player_saved_rotation);
		
		// We don't want to move when we are rewinding.
		//collision_box->SetPhysicsLinearVelocity(FVector(0));
		collision_physics->bLockXTranslation = 1;
		collision_physics->bLockYTranslation = 1;
		collision_physics->bLockZTranslation = 1;
		//collision_physics->bLockZRotation = 1;

		--state_array.world_count;
		state_array.world_memory_size -= world_state_type_memory_size;
		
		// If no states to rewind, reset and disallow to rewind.
		if (state_array.world_count < 0) {
			allowed_to_rewind = false;
			currently_rewinding = false;
			state_array.world_count = 0;
			state_array.world_memory_size = 0;
			// save_world_state_timer is reset on rewind button release.

			// Start moving with last saved speed.
			collision_physics->bLockXTranslation = 0;
			collision_physics->bLockYTranslation = 0;
			collision_physics->bLockZTranslation = 0;
			collision_box->SetPhysicsLinearVelocity(player_saved_velocity);
		}

		/*
		UE_LOG(Log_CD_Core, Log, TEXT("REWIND ============================"));
		UE_LOG(Log_CD_Core, Log, TEXT("player_state.position: %s"), *player_state.position.ToString());
		UE_LOG(Log_CD_Core, Log, TEXT("player_state.rotation: %s"), *player_state.rotation.ToString());
		UE_LOG(Log_CD_Core, Log, TEXT("player_state.velocity: %s"), *player_state.velocity.ToString());
		UE_LOG(Log_CD_Core, Log, TEXT("state_array.world_count: %d"), state_array.world_count);
		UE_LOG(Log_CD_Core, Log, TEXT("World memory: %d\t||\tRemaining memory: %d"), state_array.world_memory_size, world_array_memory_size);
		UE_LOG(Log_CD_Core, Log, TEXT("Is rewind pressed: %d"), is_time_rewind_pressed);
		*/	
	}

	// If we didn't rewind - reset timer.
	if (rewinding_timer >= 1) {
		rewinding_timer = 0;
	}

	if (is_walking) {
		A_HUI::given_time += dt * 2;
	}
}

void A_Player::move_forward() {
	is_move_forward_pressed = true;
	is_walking = true;
}

void A_Player::move_backward() {
	is_move_backward_pressed = true;
	is_walking = true;
}

void A_Player::move_right() {
	is_move_right_pressed = true;
	is_walking = true;
}

void A_Player::move_left() {
	is_move_left_pressed = true;
	is_walking = true;
}

void A_Player::jump() {
	is_jump_pressed = true;
}

void A_Player::time_rewind() {
	is_time_rewind_pressed = true;
}

void A_Player::move_forward_released() {
	is_move_forward_pressed = false;
}

void A_Player::move_backward_released() {
	is_move_backward_pressed = false;
}

void A_Player::move_right_released() {
	is_move_right_pressed = false;
}

void A_Player::move_left_released() {
	is_move_left_pressed = false;
}

void A_Player::jump_released() {
	is_jump_pressed = false;
}

void A_Player::time_rewind_released() {
	is_time_rewind_pressed = false;
	
	// Failsafe if player quit rewinding before exceeding all available rewind states.
	currently_rewinding = false;

	// I don't want timer to reset with button mashing.
	if (save_world_state_timer >= 1 + 0.084f) {
		save_world_state_timer = 0;
	}

	// Start moving with last saved speed if we not rewinding.
	FVector player_saved_velocity = state_array.world[state_array.world_count].saved_player.velocity;

	collision_physics->bLockXTranslation = 0;
	collision_physics->bLockYTranslation = 0;
	collision_physics->bLockZTranslation = 0;
	
	collision_box->SetPhysicsLinearVelocity(player_saved_velocity);
}

void A_Player::mouse_movement_x(float value) {
	//UE_LOG(Log_CD_Core, Log, TEXT("Mouse X: %.3f"), value);
	mouse_input_x = value;
}

void A_Player::mouse_movement_y(float value) {
	//UE_LOG(Log_CD_Core, Log, TEXT("Mouse Y: %.3f"), value);
	mouse_input_y = value;
}