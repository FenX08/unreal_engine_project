#include "bot.h"
#include "hero.h"
#include "hui.h"

#include "Components/SceneComponent.h"
#include "Components/BoxComponent.h" // For collision.
#include "Camera/CameraComponent.h"
#include "EngineUtils.h" // For TActorIterator.
#include "DrawDebugHelpers.h"
#include "vector" // For dynamic arrays.
#include "Kismet/GameplayStatics.h" // To get real time.

#include "cd_core/log.h"

float 	A_Bot::bot_speed 		= 0;
FVector	A_Bot::objective_vector = FVector(0);

namespace {
	float dt;

	float	collision_size 			= 20.0f;
	float 	whole_collision_size	= collision_size * 2; // Whole cuboid collision is twice collision_size.
	// Collizion Z is actually twice this number. It's height is a sum of collision_height up and down.
	// So if collision_height = 80.0f, whole collision height will be 160.0f in Unreal.
	float	collision_height 	= 92.0f;
	FVector collision_bounds(collision_size, collision_size, collision_height);
	FVector collision_velocity;

	// Intelligence variables.
	AActor 					*point_b;
	FCollisionQueryParams 	collision_parameters_for_path_search;

	FVector 				new_final_point(0);
	FVector 				current_final_point(0);
	
	std::vector<FVector> 	path_point_array;

	bool found_new_final_point 					= false;
	bool found_path 							= false;
	bool search_right 							= false;
	bool rotation_side_direction_was_randomized = false;
	bool failed_one_side_search					= false;

	struct Walking_Path_Info {
		// Zero is starting point (bot location), we want to count from 1 (first path point).
		int 	target_path_point 			= 1;
		FVector	current_path_point 			= FVector(0);
		FVector point_forward				= FVector(0);
		float 	initial_distance_to_point 	= 0;
		bool 	rotation_direction_right 	= false;
	} walking_path_info;

	bool ready_to_go_to_path_point 				= false;
	bool can_simulate_rotation 					= false;
	bool can_simulate_walking 					= false;

	struct AI_Error_Info {
		FVector	saved_bot_position					= FVector(0);
		float 	walking_error_timer 				= 1.0f;
		float 	walking_error_timer_count 			= 0.0f;
		bool	position_saved 						= false;
		int 	failed_to_search_in_some_direction 	= 0;
	} ai_error_info;
	
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
	
	// Bot move variables.
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
	
	bool 			is_walking = false;	// Right now I control this by key input and not by checking bot velocity.
	
	bool 			mass_has_no_effect = true;
	
	// @note: Hardcoded vectors for move input keys on unit circle.
	FVector			move_forward_xy		(0,1,0);
	FVector			move_backward_xy	(0,-1,0);
	FVector			move_right_xy		(1,0,0);
	FVector			move_left_xy		(-1,0,0);
	
	struct Bot_State {
		FVector position = FVector(0);
		FQuat 	rotation = FQuat::Identity;
		FVector velocity = FVector(0);
	} bot_state;
}

A_Bot::A_Bot(const FObjectInitializer &ObjectInitializer) : Super(ObjectInitializer) {
	root = ObjectInitializer.CreateDefaultSubobject<USceneComponent>(this, TEXT("Root Component"));
	SetRootComponent(root);
	
	// Bot Inputs are procceced in TG_PrePhysics,
	// so we are ensuring that our bot update executes as soon as possible.
	// We can also give different TickGroup for different bot components.
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
	
	camera->bAutoActivate 			= false; // @note: We do not activate camera for the bot.
	camera->FieldOfView 			= 104.0f;
	camera->bUsePawnControlRotation = false;
	
	// Offset camera in box collision by desired vector.
	camera->SetRelativeLocation(camera_offset);
}

void A_Bot::PostLoad() {
	Super::PostLoad();
	
	// @note: This is for the player. Ignore this for now.
	//AutoReceiveInput = EAutoReceiveInput::Player0;
}

void A_Bot::SetupPlayerInputComponent(UInputComponent *input_component) {
	Super::SetupPlayerInputComponent(input_component);
	
	// @note: IE_Repeat parameter works slow (with delay). Something to do with text typing?
	input_component->BindAction("move_forward", IE_Pressed, this, &A_Bot::move_forward);
	input_component->BindAction("move_backward", IE_Pressed, this, &A_Bot::move_backward);
	input_component->BindAction("move_right", IE_Pressed, this, &A_Bot::move_right);
	input_component->BindAction("move_left", IE_Pressed, this, &A_Bot::move_left);
	input_component->BindAction("jump", IE_Pressed, this, &A_Bot::jump);
	
	input_component->BindAction("move_forward", IE_Released, this, &A_Bot::move_forward_released);
	input_component->BindAction("move_backward", IE_Released, this, &A_Bot::move_backward_released);
	input_component->BindAction("move_right", IE_Released, this, &A_Bot::move_right_released);
	input_component->BindAction("move_left", IE_Released, this, &A_Bot::move_left_released);
	input_component->BindAction("jump", IE_Released, this, &A_Bot::jump_released);
	
	input_component->BindAxis("mouse_movement_x", this, &A_Bot::mouse_movement_x);
	input_component->BindAxis("mouse_movement_y", this, &A_Bot::mouse_movement_y);
}

void A_Bot::BeginPlay() {
	Super::BeginPlay();

	// Reset this camera vector cache for now.
	camera_euler_rotation = FRotator(0);

	for (TActorIterator<AActor> actor_iterator(GetWorld()); actor_iterator; ++actor_iterator) {
		//AActor *actor = *actor_iterator;

		// @todo: I can't use this for shipping build. Can only use for debugging.
		// Can use GetName(), but it returns no useful information about the object.
		// Epic Games calls themself a real game development company, literally.
		// Epic Games, more like Epic Farts. Sorry, not sorry.
		// I spent 1-2 hours trying this for-cycle to work, would be very cool
		// if Epic Games could provide Epic Documentation somewhere to understand
		// that you can't use this get name (label) anywhere except for debugging.
		// This method can only be used with ActorHasTag(name) and
		// assigning tag to an object root component. So good luck on not getting
		// confused where you put your fucking tag in those objects.
		if (actor_iterator->GetActorLabel() == "point_b") {
			point_b = *actor_iterator;
			UE_LOG(Log_CD_Core, Log, TEXT("Actor: %s"), *point_b->GetActorLabel());
		}
	}

	// AI stuff:
	// Ignore some collisions.
	collision_parameters_for_path_search.AddIgnoredActor(this); // Ignore bot collision.
	collision_parameters_for_path_search.AddIgnoredActor(A_Player::player); 
	reset_ai_logic();
}

void A_Bot::reset_ai_logic() {
	// Bunch of global variables.
	// @todo: Would be great if you could reduce number of global variables
	// and start to structure your data more wisely :) 
	new_final_point		= FVector(0);
	current_final_point	= FVector(0);
	
	path_point_array.clear();
	
	found_new_final_point					= false;
	found_path 								= false;
	search_right							= false;
	rotation_side_direction_was_randomized	= false;
	failed_one_side_search					= false;
	
	//Walking_Path_Info walking_path_info_reset;
	//walking_path_info = walking_path_info_reset;
	walking_path_info = Walking_Path_Info();

	ready_to_go_to_path_point				= false;
	can_simulate_rotation 					= false;
	can_simulate_walking 					= false;

	ai_error_info = AI_Error_Info();
}

void A_Bot::Tick(float dt_from_tick) {
	Super::Tick(dt_from_tick);

	dt = dt_from_tick;

	//UE_LOG(Log_CD_Core, Log, TEXT("Bot position: %s"), *GetActorLocation().ToString());

	// @note: Use this for timer measuring.
	//int32 seconds;
	//float partial_seconds;
	//UGameplayStatics::GetAccurateRealTime(GetWorld(), seconds, partial_seconds);
	//UE_LOG(Log_CD_Core, Log, TEXT("Time passed and frame time:\n%.24f\n%.24f"), new_time - current_time, dt);
	
	static bool		want_ai_timer	= false;	
	static float 	ai_timer 		= 5.0f;
	static float 	ai_timer_count 	= 0.0f;
	
	if (want_ai_timer) {
		ai_timer_count += dt;

		if (ai_timer_count >= ai_timer) {
			ai_timer_count = 0.0f;
			simulate_intelligence();
		}
	} else {
		simulate_intelligence();
	}

	move_camera();
	move_bot();
	//raycast();

	//UE_LOG(Log_CD_Core, Log, TEXT("Velocity: %s"), *collision_velocity.ToString());

	// @todo: Move this to post update maybe?
	root->SetWorldLocation(collision_box->GetRelativeLocation());
}

void A_Bot::simulate_intelligence() {
	search_rotation();
	//search_height();
	simulate_input();

	process_exceptions();
}

void A_Bot::search_rotation() {
	// @todo: What should we do, when we set last path point and we found the finish point?
	// Right now, we could find finish point, but if we found it at awkward angle, we can't
	// go to it, because we will not have enough space.

	// @todo: What about dead ends? We need to save them and start to search from first bot position
	// before dead end?
	// Or should we continue searching from dead end?
	
	// @todo: What if we found finish point, but can't actually go to it?
	// Mark this area unreachable and search the other way? Define "mark this area", haha.

	// @todo: Bot will fall, because I don't check ground. This problem is for search_height()?

	// @todo: Consider using capsule collision for bot, instead of cuboid.

	// @note: Right now the farther away the bot is (or point) from finish,
	// the less accurate the rays are.

	// @todo: I can try to profile this by counting how many raycasts I launched to
	// find final_point. I need measure how much time it takes to make one raycast.
	// After that I can count approximate time it took to find final_point:
	// sum_of_raycasts * time_for_one_raycast = time_took_to_find_final_point

	// @speed: To get extra speed I can count how many cycles I made in find_path_point(),
	// and if we didn't anything after some number of turns, save current turn degrees, and
	// continue doing different stuff. After that, he will comeback on next frame and
	// continue searching for path from saved turn degrees.

	// @todo: If we are not that far away from final_point and we need to pass one last wall,
	// we can choose for bot to decide, if he will go into different direction.
	// For example: We're searching right. final_point is near, but still behind the wall.
	// 				Start searching left from last path_point.

	// @todo: We can make two arrays of path points.
	// One will contain newly found path.
	// Second will contain the same path, but with reduced points. It will be a shorter path.
	// To find shorter path, I can iterate between n and n+2, if theres no obstacles,
	// I can remove n+1 and shift n+2 to the place of n+1.
	// Of course I can compare n with n+(2+x) to find the shortest path, but this
	// probably will be expansive.

	// @todo: If I set final point as something that is within collision, like
	// player position, this code will work wrong, because it will consider player collision
	// as wall, which we are not. If you want to make proper "follow me" feature,
	// I think you need to ignore player collision in search raycasts.
	
	// Check if we got new objective vector.
	// Objective vector is a signal that comes from somewhere else.
	// @note: 	What if there are bunch of objectives? Maybe we will need some
	// 			kind of objective queue (array)?
	//objective_vector = point_b->GetActorLocation(); // @hack: Remove this later.
	objective_vector = A_Player::player_position; // @hack: Remove this later.
	new_final_point = objective_vector;
	
	// If we didn't start searching or we reached final point, path point array will be empty.
	if (path_point_array.size() == 0) {
		if (current_final_point != new_final_point) {
			found_new_final_point 	= true;
			current_final_point 	= new_final_point;
		} else {
			found_new_final_point = false;
		}
	}

	if (!found_path && found_new_final_point) {
		// If it's the first time we are searching the path, start position will be bot's position.
		// If not, start from last point we found.
		FVector start_point;

		if (path_point_array.size() == 0) {
			path_point_array.push_back(collision_box->GetComponentLocation());

			start_point = path_point_array[0];
		} else {
			start_point = path_point_array[path_point_array.size() - 1];
		}
		
		// @note: What will happen if final point will change mid path finding?
		FVector final_point 			= current_final_point;
				final_point.Z			= start_point.Z;
		FVector start_to_final			= final_point - start_point;
		float 	start_to_final_distance	= start_to_final.Size2D();

		// Raycast from start to final point.
		FHitResult 	out_hit_main;
		bool 		found_final_point;
		bool 		got_hit_main = GetWorld()->LineTraceSingleByChannel(out_hit_main, start_point, final_point, ECC_Visibility, collision_parameters_for_path_search);
		
		// If we found obstacle between start and final_point, search where to go.
		if (got_hit_main) {
			found_final_point = false;
			find_path_point(start_point, start_to_final, start_to_final_distance, found_final_point);
		} else { // No obstacles found and we are "looking" straight at final_point.
			// We didn't actually found path, we just found (saw) final point.
			// We will do extra logic to know that we can actually reach final point and
			// declare that we found path.
			found_final_point = true;
			
			// We need to perform last check to see if bot can actually reach final_point
			// using two raycasts from cuboid collision of bot.
			FVector last_point 				= start_point;
			FVector last_to_final			= final_point - last_point;
			//float 	last_to_final_distance	= last_to_final.Size2D();

			// Find angle between last path point and final_point and cosine vector on trigonometric circle for
			// two raycasts from cuboid collision of bot.
			FVector last_to_final_search 	= last_to_final;
					last_to_final_search.Z 	= 0;
			last_to_final_search.Normalize();
			
			float last_to_final_angle = FMath::Acos(FVector::DotProduct(last_to_final_search, FVector(1,0,0)));
			if (last_to_final_search.Y < 0) {
				last_to_final_angle = tau - last_to_final_angle;
			}
			
			// Find front corners of cuboid bot collision (front is looking at final_point direction from new extra path_point).
			// We use this to know that we could actually reach final_point.
			float length_of_hypotenuse_from_center_of_bot_collision = FMath::Sqrt(2) * collision_size;
			
			// Find front left corner of cuboid.
			float 	front_left_corner_angle = last_to_final_angle - pi/4; // Minus degrees on trig circle is left for Unreal top-down view.
			FVector front_left_corner(0,0,0);
			
			FMath::SinCos(&front_left_corner.Y, &front_left_corner.X, front_left_corner_angle);
			front_left_corner 	*= length_of_hypotenuse_from_center_of_bot_collision;
			front_left_corner 	+= last_point;
						
			// Find front right edge.
			float 	front_right_corner_angle = last_to_final_angle + pi/4; // Plus degrees on trig circle is right for Unreal top-down view.
			FVector front_right_corner(0,0,0);
			
			FMath::SinCos(&front_right_corner.Y, &front_right_corner.X, front_right_corner_angle);
			front_right_corner 	*= length_of_hypotenuse_from_center_of_bot_collision;
			front_right_corner 	+= last_point;
						
			// Find vector for side shifting in corner trace cycle.
			// If we are serching right side, shift will be to the right relative to the final_point.
			// And vice versa.
			float side_shift_angle;
			if (search_right) {
				side_shift_angle = last_to_final_angle + pi/2; // Plus degrees on trig circle is right for Unreal top-down view.
			} else {
				side_shift_angle = last_to_final_angle - pi/2; // Minus degrees on trig circle is left for Unreal top-down view.
			}
			
			FVector side_shift_vector(0,0,0);
			FMath::SinCos(&side_shift_vector.Y, &side_shift_vector.X, side_shift_angle);
			
			// We will shift by 1.5 of whole cuboid collision.
			side_shift_vector *= collision_size * 3;

			// Corner trace cycle. Find if we can actually reach final_point.
			// If not, shift number of times and if we can actually reach final_point,
			// set new path_point. If shifting was not successful,
			// do not set new path_point and just leave.
			FVector 	new_path_point 				= last_point;
			FVector 	new_to_final				= last_to_final;
			FVector 	final_point_for_left(0);
			FVector 	final_point_for_right(0);
			FHitResult 	out_hit_left;
			FHitResult 	out_hit_right;
			bool		made_side_shift				= false;
			bool 		path_is_clear 				= false;
			
			// @note: On what this number depends? On bot collision?
			// Will it stay as magic number?
			int times_to_shift_to_the_side = 1;
			
			// @todo: Raycast to the side to see that we can actually side shift.
			for (int i = 1; i <= times_to_shift_to_the_side + 1; ++i) {
				if (i > 1) {
					front_left_corner 	+= side_shift_vector;
					front_right_corner	+= side_shift_vector;
					new_path_point		+= side_shift_vector;
					new_to_final		= final_point - new_path_point;
				}
				
				// Raycast from left and right side of cuboid collision straight at final point direction.
				// @note: If it's first iteration new_to_final is just last_to_final.
				final_point_for_left 	= front_left_corner + new_to_final;
				final_point_for_right 	= front_right_corner + new_to_final;
				
				bool got_hit_left 	= GetWorld()->LineTraceSingleByChannel(out_hit_left, front_left_corner, final_point_for_left, ECC_Visibility, collision_parameters_for_path_search);
				bool got_hit_right 	= GetWorld()->LineTraceSingleByChannel(out_hit_right, front_right_corner, final_point_for_right, ECC_Visibility, collision_parameters_for_path_search);
				
				// Draw forward vector from corners to final_point direction.
				//DrawDebugLine(GetWorld(), front_left_corner, final_point_for_left, FColor::Black, false, 10000.0f, 0, 1.2f);
				//DrawDebugLine(GetWorld(), front_right_corner, final_point_for_right, FColor::Black, false, 10000.0f, 0, 1.2f);
				
				// If traces weren't colliding with anything, path is clear.
				if (!got_hit_left && !got_hit_right) {
					if (i > 1)
						made_side_shift = true;
					
					path_is_clear = true;
					break;
				}
			}

			// We need to set new path point and rewrite start point if we made a shift.
			if (made_side_shift) {
				set_path_point(start_point, new_path_point);
				start_point = new_path_point;
			}

			// If both rays didn't hit anything - we have finally found the path!
			if (path_is_clear) {
				found_path = true;

				// Put final point at the end of array to use it for movement logic.
				path_point_array.push_back(final_point);

				// Draw line from last path_point to final_point to indicate that we can reach final_point.
				DrawDebugLine(GetWorld(), start_point, final_point, FColor::Green, false, 10000.0f, 0, 1.2f);
			} else {
				// Draw line from last path_point to final_point to indicate that we saw final_point,
				// but we can't reach.
				DrawDebugLine(GetWorld(), start_point, final_point, FColor::Red, false, 10000.0f, 0, 1.2f);

				// We got some obstacles, find new point with found_final_point exception
				// and on to the next frame.
				find_path_point(start_point, start_to_final, start_to_final_distance, found_final_point);
			}
		}
	}
}

void A_Bot::find_path_point(FVector start_point, FVector start_to_final, float start_to_final_distance, bool found_final_point) {	
	FVector path_point(0);
	
	// @todo: 	Make sure to make implementation, when we don't hit any collision
	// 			from A to B.
	// Also, if we found straight path, there is no guarantee that we can fit into this
	// straight passage.
	
	// @todo: 	What if there is an obstacle at the bottom or at the top of bot collision?
	// 			We need something like this: check_height_for_collision(dt);
	// We will need this to know if collision can fit into found passage.
	
	// @hack: To find if we can fit into passage, right now I'll some big width number.
	// 		Maybe I can use more than twice of collision, before I implement something
	// like check_height_for_collision(), so that bot wouldn't stuck at some
	// obstacles or tricky wall angles, because I don't use Z axes in rotation search.
	// 		Yeah, seems like we need to use more than twice of the collision.
	// Right now my trace precision is only 360 degrees. Sometimes there is more wall
	// collision left and we need to account for that.
	// Will think about it when I'll test extreme cases like corridor with same width
	// as bot collision. Guess right now I'll hack my way through that.
	// -- Richard Chirkin 22.01.2022
	float safe_distance_to_pass = whole_collision_size * 3;
	
	// Point B is like the center of a circle. start_to_final_distance is radius of a circle.
	// But we will use diameter of this imaginary B circle for our path search.
	// @todo: 	What if target is near and passage to target is very far?
	// 			Will this still work? Think not.
	float search_length = start_to_final_distance * 2;
	
	// We are searching only in XY plane, we don't need Z.
	FVector start_to_final_search 	= start_to_final;
	start_to_final_search.Z 		= 0;
	start_to_final_search.Normalize();
	
	// Prepare everything for raycast cycle.
	FVector	new_search_vector(0);
	FVector	first_success_trace_vector(0);
	
	float 	new_trace_distance 		= 0;
	float 	last_hit_distance 		= 0;
	//float 	new_normal_angle	= 0; // I'm not using normal angles for now... Maybe never?
	//float 	old_normal_angle	= 0;
	int 	success_traces_count 	= 0;
	
	bool 	found_passage = false;
	
	// Randomize decision in what direction to go.
	if (!rotation_side_direction_was_randomized) {
		rotation_side_direction_was_randomized = true;
		
		int random_number = FMath::RandRange(0, 1);
		
		if (random_number == 0) {
			search_right = false;
		} else {
			search_right = true;
		}
	}
	
	// Find angle between start_to_final_search and cosine vector on trigonometric circle, to know
	// where to rotate in search cycle.
	float start_to_final_search_angle = FMath::Acos(FVector::DotProduct(start_to_final_search, FVector(1,0,0)));
	if (start_to_final_search.Y < 0) {
		start_to_final_search_angle = tau - start_to_final_search_angle;
	}
	
	int maximum_to_rotate = 360;
	
	for (int i = 0; i <= maximum_to_rotate; ++i) {
		// If we add degrees it's rotation to the right in Unreal, if we substract - it's rotation to the left.
		float search_angle = start_to_final_search_angle * (180/pi);
		
		// We rotate and search by one degree, not radians.
		if (search_right) {
			search_angle += i;
		} else {
			search_angle -= i;
		}
		
		search_angle *= pi/180; // Convert back to radians.
		
		//UE_LOG(Log_CD_Core, Log, TEXT("Angle: %.2f"), start_to_final_search_angle * (180.f/pi));
		
		FVector search_vector(0,0,0);
		FMath::SinCos(&search_vector.Y, &search_vector.X, search_angle);
		new_search_vector 	= search_vector;
		search_vector 		*= search_length;
		
		// We move found vector to start_point to use for raycast.
		search_vector += start_point;
		
		// Line trace search.
		FHitResult 	out_hit_search;
		FVector 	point_hit(0);
		FVector 	point_hit_normal(0);
		bool		found_empty_space = false;

		bool got_hit_search = GetWorld()->LineTraceSingleByChannel(out_hit_search, start_point, search_vector, ECC_Visibility, collision_parameters_for_path_search);
		
		if (got_hit_search) {
			if (out_hit_search.bBlockingHit) {
				point_hit 			= out_hit_search.ImpactPoint;
				point_hit_normal 	= out_hit_search.ImpactNormal;
				/*	
					// We don't need Z for angle search.
					point_hit_normal.Z = 0;
					
					// Find angle for current line.
					new_normal_angle = FMath::Acos(FVector::DotProduct(point_hit_normal, FVector(1,0,0)));
					if (point_hit_normal.Y < 0) {
					new_normal_angle = tau - new_normal_angle;
					}
					*/	
				// Distance of current line.
				new_trace_distance = out_hit_search.Distance;
			}
		} else { // Found empty space (no walls were hit).
			found_empty_space 	= true;
			new_trace_distance 	= search_length;
		}
		
		// If this is initial trace, just save information and continue.
		if (i == 0) {
			// If we are looking at final_point, take twice of whole collision
			// as distance for first trace, to compare it against success traces.
			if (found_final_point) {
				last_hit_distance = whole_collision_size * 2;
			} else {
				last_hit_distance = new_trace_distance;
			}
			
			new_trace_distance 	= 0.0f;
			
			// If first trace, draw line towards point b location.
			DrawDebugLine(GetWorld(), start_point, point_hit, FColor::Yellow, false, dt + 0.0001f, 0, 1.2f);
			
			continue;
		}
		
		// Find out if our trace was successful to meet requirements on how to be successful or
		// if it failed miserably.
		bool success_trace = false;
		if (new_trace_distance > last_hit_distance) {
			// New wall is further.
			// If this is an opening we fit in 1D, than it's a success trace.
			if (new_trace_distance - last_hit_distance >= safe_distance_to_pass) {
				// @todo: There is probably passage between walls. Should I do normal angle checking?
				success_trace = true;
			} else {
				success_trace = false;
			}
		} else {
			// New wall is nearer or the same. No opening, sadge. 
			success_trace = false;
			
			// If we move in space, we need to find depth (opening) to go through.
			// But even if the wall seems nearer with no opening in 1D (in 1D wall = dot),
			// there is still can be opening in 2D between last_hit_distance's wall and new wall.
			if (new_trace_distance - last_hit_distance <= -safe_distance_to_pass) {
				// @todo: There is probably passage between walls. Should I do normal angle checking?
			}
		}
		
		if (success_trace) {
			// We are hoping that we get successful sequence of traces and we don't
			// care how far away new success traces are from last_hit_distance
			// (or more like how longer are they from last_hit_distance).
			// If we fulfill passage width with continues success traces, we will use
			// last_hit_distance to find where we should place path point.
			// That's why we don't overwrite last_hit_distance in success_trace.
			FVector new_trace_vector = new_search_vector * last_hit_distance;
			
			// We count success traces to know when was the first one and save it's search vector
			// to use for point finding. We are finding the point in between the first and the last success trace.
			++success_traces_count;
			if (success_traces_count == 1) {
				first_success_trace_vector = new_trace_vector;
			}
			
			float distance_between_line_traces = FVector::Distance(new_trace_vector, first_success_trace_vector);
			
			if (!found_empty_space) {
				// Draw success trace.
				DrawDebugLine(GetWorld(), start_point, point_hit, FColor::Green, false, dt + 0.0001f, 0, 1.2f);
				
				// Draw normal of success trace. If there's no wall, no normal will be drawn. Remember, normal's Z is zero.
				//DrawDebugLine(GetWorld(), point_hit, point_hit + point_hit_normal * 200.0f, FColor::White, false, dt + 0.0001f, 0, 1.2f);
			} else {
				// Draw success trace with the length of search_vector.
				DrawDebugLine(GetWorld(), start_point, search_vector, FColor::Green, false, dt + 0.0001f, 0, 1.2f);
			}
			
			// If opening is wide enough to go through, guess we found the passage!
			// @todo: 	If wall is very near and we traced more than 180 degrees (if 180 is max), path point will not be set,
			// 			because we are setting it using last_hit_distance that is very near us, and
			// 			distance_between_line_traces will always be smaller than safe_distance_to_pass,
			// 			but nevertheless there is actually enough space to set a point.
			// Is this because safe_distance_to_pass is not actual bot collision and just a hack,
			// or I need to set path point differently? Or just use 360 degrees?
			if (distance_between_line_traces >= safe_distance_to_pass) {
				found_passage = true;
				
				// We want half of perpendicular vector from first_success_trace_vector to new_trace_vector.
				float half_distance_between_new_trace_and_last_hit = distance_between_line_traces / 2;
				FVector vector_to_set_point = new_trace_vector - first_success_trace_vector;
				vector_to_set_point.Normalize();
				vector_to_set_point *= half_distance_between_new_trace_and_last_hit;
				
				path_point = start_point + first_success_trace_vector + vector_to_set_point;
				
				break;
			}
		} else {
			if (i > 0) { // If not first trace.
				// Save info for next success_trace comparison.
				last_hit_distance = new_trace_distance;
				
				// Reset success traces count.
				success_traces_count = 0;
				
				// Draw fail trace.
				DrawDebugLine(GetWorld(), start_point, point_hit, FColor::Red, false, dt + 0.0001f, 0, 1.2f);
				
				// Draw wall normal of failed trace. Remember, normal's Z is zero.
				//DrawDebugLine(GetWorld(), point_hit, point_hit + point_hit_normal * 200.0f, FColor::Blue, false, dt + 0.0001f, 0, 1.2f);
			}
		}

		// If we didn't find the passage, for now just rotate search other side and
		// hope that someday we will find the passage.
		if (i == maximum_to_rotate) {
			search_right = !search_right;

			// Save info that we failed to do rotation search in one direction.
			++ai_error_info.failed_to_search_in_some_direction;

			//UE_LOG(Log_CD_Core, Log, TEXT("Bot rotated 360 degrees and found nothing! search_right was: %d"), search_right);
			
			// @note: Stop searching if we didn't found anything in right and left turns?
			// @note: If we failed searching in both directions, we never stop searching.
			// Do we actually need to stop? EZ
			// 360 degrees of raycasts will make your CPU very happy every frame! GIGACHAD

			// @todo: This looks like the case where we actually found a dead end!

			// @todo: 	If you place a point, that bot can't actually go to (not just Z height in 3D),
			// bot will do rotation search forever.
			// This is caused by safe_distance_to_pass being too high that results in
			// bot being stuck forever.
			// There is just not enough space for bot to get to our final point Sadge
			// @note: 	Guess you can mark final point unreachable and just wait for another
			// objective or maybe find new objective that you can reach.
		}
	}
	
	if (found_passage) {
		set_path_point(start_point, path_point);
	}
}

void A_Bot::set_path_point(FVector start_point, FVector path_point) {
	path_point_array.push_back(path_point);
	
	FColor pinkish(228, 80, 162); // Pinkish.
	FColor turquoise(71, 226, 239); // Victory turquoise.
	
	// Draw where point is.
	FVector path_point_ground(path_point.X, path_point.Y, start_point.Z - collision_height);
	FVector path_point_ceiling(path_point.X, path_point.Y, start_point.Z + collision_height);
	DrawDebugLine(GetWorld(), path_point_ground, path_point_ceiling, pinkish, false, 10000.0f, 0, 1.2f);
	
	// Draw line from point to point.
	DrawDebugLine(GetWorld(), start_point, path_point, turquoise, false, 10000.0f, 0, 1.2f);
}

void A_Bot::search_height() {
	// We need to search for descent, by searching it from point B (target)
	// to the plane where A (bot) stands.
	// We can try to look at the whole level from side and rotate it (imaginary) by 90 degrees to the left.
	// Then we will be sure that descent will be always on the right.
}

void A_Bot::simulate_input() {
	// @todo: Start moving when we found at least one path point and not just whole path.
	if (found_path && !ready_to_go_to_path_point) {
		// Initialize stuff before doing rotation and walking.
		FVector bot_position 	= collision_box->GetRelativeLocation();
		FVector bot_forward 	= collision_box->GetForwardVector();
		FVector path_point		= path_point_array[walking_path_info.target_path_point];
		FVector bot_to_point	= path_point - bot_position;
		
		walking_path_info.current_path_point 		= path_point;
		walking_path_info.initial_distance_to_point = bot_to_point.Size2D(); // do we need this?

		bot_to_point.Z = 0;
		bot_to_point.Normalize();
		walking_path_info.point_forward	= bot_to_point;
		
		// If number of cross Z is zero and dot product is less than zero,
		// it means our bot and point vectors are looking at opposite directions.
		// In this case, just randomize decision in what direction to rotate.
		// @note: 	Later this decision can be based on whether or not we are
		// 			near some wall and we want rotate to in opposite direction from wall.
		bool 	rotation_direction_right 	= false;
		bool	direction_was_randomized	= false;
		FVector	point_forward				= bot_to_point;
		FVector cross_vector 				= FVector::CrossProduct(bot_forward, point_forward);
		float	cross_z 					= cross_vector.Z;
		float 	dot_product					= FVector::DotProduct(bot_forward, point_forward);
		float 	epsilon 					= 0.001f;
		
		if (FMath::IsNearlyEqual(cross_z, 0.0f, epsilon))
		{
			if (dot_product < 0.0f)
			{
				direction_was_randomized 	= true;
				int random_number 			= FMath::RandRange(0, 1);

				if (random_number == 0) {
					rotation_direction_right = false;
				} else {
					rotation_direction_right = true;
				}
			} // else, they actually look at the same direction,
			  // we can ignore everything and continue, bot will not make
			  // any rotation and eventually will go straight to the walking.
		}

		// Choose in what direction to rotate.
		if (!direction_was_randomized) {
			if (cross_z > 0.0f) { 
				rotation_direction_right = true;
			} else {
				rotation_direction_right = false;
			}
		}

		walking_path_info.rotation_direction_right = rotation_direction_right;

		ready_to_go_to_path_point 	= true;
		can_simulate_rotation		= true;
	}
	
	// We rotate first.
	// @todo: Start rotating to the next path point when we are near current path point.
	if (can_simulate_rotation)
		simulate_rotation();
	
	// After rotation is done - we walk.
	if (can_simulate_walking)
		simulate_walking();

	//UE_LOG(Log_CD_Core, Log, TEXT("can_simulate_walking: %d"), can_simulate_walking);
}

void A_Bot::simulate_rotation() {
	bool 	rotation_direction_right 	= walking_path_info.rotation_direction_right;
	float 	rotation_speed				= 3.0f;

	// We are rotating only horizontally for now.
	if (rotation_direction_right) {
		mouse_input_x = rotation_speed;
	} else {
		mouse_input_x = -rotation_speed;
	}

	FVector bot_forward 	= collision_box->GetForwardVector();
	FVector point_forward	= walking_path_info.point_forward;
	float 	dot_product		= FVector::DotProduct(bot_forward, point_forward);
	float	epsilon 		= 0.001f;

	//UE_LOG(Log_CD_Core, Log, TEXT("dot_product: %.2f"), dot_product);

	// If bot is looking straight at path point, we can walk to it.
	if (FMath::IsNearlyEqual(dot_product, 1.0f, epsilon)) {
		can_simulate_rotation 	= false;
		can_simulate_walking 	= true;

		mouse_input_x = 0.0f;
	}
}

void A_Bot::simulate_walking() {
	is_walking 				= true;
	is_move_forward_pressed = true;
	
	FVector bot_position 	= collision_box->GetRelativeLocation();
	FVector bot_forward 	= collision_box->GetForwardVector();
	FVector path_point		= walking_path_info.current_path_point;
	float 	epsilon 		= 0.001f;
	
	// Eсли у бота есть полуугол зрения по горизонтали fov, то бот видит тебя
	// -- Mr_indieperson
	// dot(botDir, normalize(targetPos - botPos)) > cos(fov) 
	
	// @todo: If we are moving very slow, think we are stuck at wall.
	// Maybe start to find new path if out position hasn't changed that much
	// in 1-2 seconds?

	// Check if bot reached path point.
	// Also extra check if bot was moving at high speed and right now is past path point
	// that is currently behind bot.
	if (bot_position.Equals(path_point, epsilon)
		|| FVector::DotProduct(bot_forward, path_point - bot_position) < 0.0f) {
		is_walking 					= false;
		is_move_forward_pressed 	= false;
		
		can_simulate_walking 		= false;
		ready_to_go_to_path_point 	= false;
		
		++walking_path_info.target_path_point;
		
		if (walking_path_info.target_path_point > path_point_array.size() - 1) {
			// We reached final point! You can now wait for new objective!
			
			// Reset path.
			walking_path_info.target_path_point = 1; // Count from one in next walking simulation.

			// @note: If level changes dynamically we do not want to save found path.
			path_point_array.clear();

			found_path 								= false;
			rotation_side_direction_was_randomized	= false;
		}
	}
}

void A_Bot::process_exceptions() {
	FVector bot_position = collision_box->GetRelativeLocation();

	// As a precaution, if we failed to find path point in both direction, just reset.
	if (ai_error_info.failed_to_search_in_some_direction == 2) {
		UE_LOG(Log_CD_Core, Log, TEXT("Bot searched right and left and didn't found the passage! Path point was not set! Resetting bot's AI. Bot position: %s"), *bot_position.ToString());
		reset_ai_logic();
	}

	if (found_path) {
		// We want to check for case, when bot stopped moving for some unknown reason.
		// Maybe he stuck at the wall, or something moved him at awkward spot.
		// Just test if we are still moving after 1 second, and if we are not - reset AI.
		// @note: Make the same if there was an error in raycasts?
		FVector &last_position 	= ai_error_info.saved_bot_position;
		float	walking_timer	= ai_error_info.walking_error_timer;
		float	&timer_count	= ai_error_info.walking_error_timer_count;
		
		if (!ai_error_info.position_saved) {
			ai_error_info.position_saved = true;
			last_position = bot_position;
			timer_count = 0.0f;
		}
		
		timer_count += dt;
		
		if (timer_count >= walking_timer) {
			timer_count = 0.0f;
			ai_error_info.position_saved = false;
			
			// @todo: Play specific animation when this error occurred?
			float epsilon = 1.0f; // are you sure that 1 cm is enough?
			if (bot_position.Equals(last_position, epsilon)) {
				UE_LOG(Log_CD_Core, Log, TEXT("Bot stuck! He tried to walk, but he didn't move after %.2f seconds! Resetting bot's AI. Bot position: %s"), walking_timer, *bot_position.ToString());
				reset_ai_logic();
			}
		}
	}
}

void A_Bot::move_camera() {
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

void A_Bot::move_bot() {
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
		// I do that to bot collision so that it perfectly translates from Unreal XY bottom-up view to my XY top-down unit circle.
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
		// but for now I can't reason this idea quite well into words. -- r.chirkin 12.01.2022
		collision_angle -= half_pi;

		// We add corrected bot collision angle to input angle on unit circle and we get correct angle for bot walking direction.
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
	
	// I'm taking current bot velocity, inverse it and multiply it by desired drag force.
	if (!is_walking) {
		collision_physics->AddImpulse(-collision_velocity * (drag_stop_walking_force * dt), mass_has_no_effect);
	} else {
		collision_physics->AddImpulse(-collision_velocity * (drag_walking_force * dt), mass_has_no_effect);
	}

	// We can know bot speed by finding magnitude (length, e.g. speed) in velocity vector.
	// This speed includes Z height velocity.
	collision_velocity = collision_physics->GetUnrealWorldVelocity();
	bot_speed = collision_velocity.Size();
}

void A_Bot::raycast() {
	FHitResult 				out_hit;
	FCollisionQueryParams 	collision_parameters;
	
	// Ignore bot collision.
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
			
			//UE_LOG(Log_CD_Core, Log, TEXT("Vector Z of hit: %.8f"), point_hit.Z);

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

void A_Bot::move_forward() {
	is_move_forward_pressed = true;
	is_walking = true;
}

void A_Bot::move_backward() {
	is_move_backward_pressed = true;
	is_walking = true;
}

void A_Bot::move_right() {
	is_move_right_pressed = true;
	is_walking = true;
}

void A_Bot::move_left() {
	is_move_left_pressed = true;
	is_walking = true;
}

void A_Bot::jump() {
	is_jump_pressed = true;
}

void A_Bot::move_forward_released() {
	is_move_forward_pressed = false;
}

void A_Bot::move_backward_released() {
	is_move_backward_pressed = false;
}

void A_Bot::move_right_released() {
	is_move_right_pressed = false;
}

void A_Bot::move_left_released() {
	is_move_left_pressed = false;
}

void A_Bot::jump_released() {
	is_jump_pressed = false;
}

void A_Bot::mouse_movement_x(float value) {
	//UE_LOG(Log_CD_Core, Log, TEXT("Mouse X: %.3f"), value);
	mouse_input_x = value;
}

void A_Bot::mouse_movement_y(float value) {
	//UE_LOG(Log_CD_Core, Log, TEXT("Mouse Y: %.3f"), value);
	mouse_input_y = value;
}