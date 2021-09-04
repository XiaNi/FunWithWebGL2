import {Vec3,Quat,Transform }	from "./fungi/maths/Maths.js";

// Read the current pose of a Rig then compute data to be saved to IK Pose.
export class IKCompute{
  static run( e, ik_pose ){
    let rig = e.IKRig;
    rig.pose.update_world();	// First thing, We need to compute WS Transforms for all the bones.
    App.Debug.reset();			// For this example, Lets reset visual debug for every compute.

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    this.hip( rig, ik_pose );

    this.limb( rig.pose, rig.chains.leg_l, ik_pose.leg_l );
    this.limb( rig.pose, rig.chains.leg_r, ik_pose.leg_r );

    this.look_twist( rig, rig.points.foot_l, ik_pose.foot_l, Vec3.FORWARD, Vec3.UP ); // Look = Fwd, Twist = Up
    this.look_twist( rig, rig.points.foot_r, ik_pose.foot_r, Vec3.FORWARD, Vec3.UP );

    this.spine( rig, rig.chains.spine, ik_pose, Vec3.UP, Vec3.FORWARD );

    this.limb( rig.pose, rig.chains.arm_l, ik_pose.arm_l );
    this.limb( rig.pose, rig.chains.arm_r, ik_pose.arm_r );

    this.look_twist( rig, rig.points.head, ik_pose.head, Vec3.FORWARD, Vec3.UP );
  }

  static hip( rig, ik_pose ){
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // First thing we need is the Hip bone from the Animated Pose
    // Plus what the hip's Bind Pose as well.
    // We use these two states to determine what change the animation did to the tpose.
    let b_info	= rig.points.hip,					// Rig Hip Info
      pose 	= rig.pose.bones[ b_info.idx ],		// Animated Pose Bone
      bind 	= rig.tpose.bones[ b_info.idx ];	// TPose Bone

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Lets create the Quaternion Inverse Direction based on the
    // TBone's World Space rotation. We don't really know the orientation
    // of the bone's starting rotation and our targets will have their own
    // orientation, so by doing this we can easily say no matter what the
    // default direction of the hip, we want to say all hips bones point
    // at the FORWARD axis and the tail of the bone points UP.
    let q_inv 		= Quat.invert( bind.world.rot ),				// This part can be optimized out and Saved into the Rig Hip's Data.
      alt_fwd		= Vec3.transform_quat( Vec3.FORWARD, q_inv ),
      alt_up		= Vec3.transform_quat( Vec3.UP, q_inv );

    let pose_fwd 	= Vec3.transform_quat( alt_fwd, pose.world.rot ),
      pose_up 	= Vec3.transform_quat( alt_up, pose.world.rot );

    /* VISUAL DEBUG TPOSE AND ANIMATED POSE DIRECTIONS
    let pos = pose.world.pos.clone().add( [0,0,0.1] );
    App.Debug.ln( pos, Vec3.add(pos, Vec3.FORWARD), "white" );
    App.Debug.ln( pos, Vec3.add(pos, Vec3.UP), "white" );
    App.Debug.ln( pos, Vec3.scale( pose_fwd, 0.8 ).add( pos ), "orange" );
    App.Debug.ln( pos, Vec3.scale( pose_up, 0.8 ).add( pos ), "orange" );
    */

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // With our directions known between our TPose and Animated Pose, Next we
    // start to calculate the Swing and Twist Values to swing our TPose into
    // The animation direction

    const bind_fwd = Vec3.transform_quat( Vec3.FORWARD, bind.world.rot )
    let swing = Quat.unit_vecs( bind_fwd, pose_fwd )	// First we create a swing rotation from one dir to the other.
      .mul( bind.world.rot );		// Then we apply it to the TBone Rotation, this will do a FWD Swing which will create
    // a new Up direction based on only swing.
    let swing_up	= Vec3.transform_quat( Vec3.UP, swing ),
      twist		= Vec3.angle( swing_up, pose_up );		// Swing + Pose have same Fwd, Use Angle between both UPs for twist

    /* VISUAL DEBUG SWING AND ANIMATED POSE DIRECTIONS
    let pos 		= pose.world.pos.clone().add( [0,0,0.1] );
    let swing_fwd	= Vec3.transform_quat( Vec3.FORWARD, swing );
    App.Debug.ln( pos, Vec3.scale( pose_fwd, 1.5 ).add( pos ), "white" );	// Out Swing FWD Matches Animated Pose Forward
    App.Debug.ln( pos, Vec3.scale( swing_fwd, 1.3 ).add( pos ), "orange" );
    App.Debug.ln( pos, Vec3.scale( pose_up, 1.5 ).add( pos ), "white" );	// Now we see the TPose Up Direction in its Swing Rotation
    App.Debug.ln( pos, Vec3.scale( swing_up, 1.5 ).add( pos ), "orange" );	// Both UPs share the same forward, resulting in a "Twist" Difference.
    */

    if( twist <= (0.01 * Math.PI / 180) ){
      twist = 0; // If Less the .01 Degree, dont bother twisting.
    }else{
      // The difference between Pose UP and Swing UP is what makes up our twist since they both
      // share the same forward access. The issue is that we do not know if that twist is in the Negative direction
      // or positive. So by computing the Swing Left Direction, we can use the Dot Product to determine
      // if swing UP is Over 90 Degrees, if so then its a positive twist else its negative.
      let swing_lft = Vec3.cross( swing_up, pose_fwd );
      // App.Debug.ln( pos, Vec3.scale( swing_lft, 1.5 ).add( pos ), "orange" );
      if( Vec3.dot( swing_lft, pose_up ) >= 0 ) twist = -twist;
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Save all the info we need to our IK Pose.
    ik_pose.hip.bind_height	= bind.world.pos.y;	// The Bind Pose Height of the Hip, Helps with scaling.
    ik_pose.hip.movement.from_sub( pose.world.pos, bind.world.pos );	// How much movement did the hip do between Bind and Animated.
    ik_pose.hip.dir.copy( pose_fwd );	// Pose Forward is the direction we want the Hip to Point to.
    ik_pose.hip.twist = twist;	// How Much Twisting to Apply after pointing in the correct direction.

    console.log('ik_pose.hip', ik_pose.hip)
  }

  static limb( pose, chain, ik_limb  ){
    // Limb IK tends to be fairly easy to determine. What you need is the direction the end effector is in
    // relation to the beginning of the limb chain, like the Shoulder for an arm chain. After you have the
    // direction, you can use it to get the distance. Distance is important to create a scale value based on
    // the length of the chain. Since an arm or leg's length can vary between models, if you normalize the
    // distance, it becomes easy to translate it to other models. The last bit of info is we need the direction
    // that the joint needs to point. In this example, we precompute the Quaternion Inverse Dir for each chain
    // based on the bind pose. We can transform that direction with the Animated rotation to give us where the
    // joint direction has moved to.

    let boneA	= pose.bones[ chain.first() ],	// First Bone
      boneB	= pose.bones[ chain.end_idx ],	// END Bone, which is not part of the chain (Hand,Foot)
      ab_dir	= Vec3.sub( boneB.world.pos, boneA.world.pos ),	// Direction from First Bone to Final Bone ( IK Direction )
      ab_len	= ab_dir.len();									// Distance from First Bone to Final Bone

    /* VISUAL DEBUG CHAIN POINTS
    App.Debug.pnt( boneA.world.pos, "green", 0.06, 6 );
    App.Debug.pnt( boneB.world.pos, "red", 0.06, 6 );
    App.Debug.ln( boneA.world.pos, boneB.world.pos, "green", "red", true );
    */

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Compute the final IK Information needed for the Limb
    ik_limb.len_scale = ab_len / chain.len;	// Normalize the distance base on the length of the Chain.
    ik_limb.dir.copy( ab_dir.norm() );		// We also normalize the direction to the end effector.

    // We use the first bone of the chain plus the Pre computed ALT UP to easily get the direction of the joint
    let j_dir	= Vec3.transform_quat( chain.alt_up, boneA.world.rot );
    let lft_dir	= Vec3.cross( j_dir, ab_dir );					// We need left to realign up
    ik_limb.joint_dir.from_cross( ab_dir, lft_dir ).norm(); 	// Recalc Up, make it orthogonal to LEFT and FWD

    /* VISUAL DEBUG THE DIRECTIONS BEING COMPUTED
    App.Debug.ln( boneA.world.pos, Vec3.scale( j_dir, 0.5 ).add( boneA.world.pos ), "white" ); 				// The actual Direction the first bone is pointing too (UP)
    App.Debug.ln( boneA.world.pos, Vec3.scale( lft_dir, 0.5 ).add( boneA.world.pos ), "orange" );			// the Cross of UP and FWD
    App.Debug.ln( boneA.world.pos, Vec3.scale( ab_dir, 0.5 ).add( boneA.world.pos ), "orange" );			// Dir to End Effector
    App.Debug.ln( boneA.world.pos, Vec3.scale( ik_limb.joint_dir, 0.5 ).add( boneA.world.pos ), "orange" );	// Recalc UP to make it orthogonal
    */
  }

  static look_twist( rig, b_info, ik, look_dir, twist_dir ){
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    let pose = rig.pose.bones[ b_info.idx ],	// Animated Pose Bone
      bind = rig.tpose.bones[ b_info.idx ];	// TPose Bone

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // First compute the Quaternion Invert Directions based on the Defined
    // Directions that was passed into the function. Most often, your look
    // direction is FORWARD and the Direction used to determine twist is UP.
    // But there are times we need the directions to be different depending
    // on how we view the bone in certain situations.
    let q_inv 		    = Quat.invert( bind.world.rot ),
      alt_look_dir	= Vec3.transform_quat( look_dir, q_inv ),
      alt_twist_dir	= Vec3.transform_quat( twist_dir, q_inv );

    let pose_look_dir 	= Vec3.transform_quat( alt_look_dir, pose.world.rot ),
      pose_twist_dir 	= Vec3.transform_quat( alt_twist_dir, pose.world.rot );

    /* VISUAL DEBUG TPOSE AND ANIMATED POSE DIRECTIONS
    let pos = pose.world.pos.clone().add( [0,0,0.0] );
    App.Debug.ln( pos, Vec3.add(pos, look_dir), "white" );
    App.Debug.ln( pos, Vec3.add(pos, twist_dir), "white" );
    App.Debug.ln( pos, Vec3.scale( pose_look_dir, 0.8 ).add( pos ), "orange" );
    App.Debug.ln( pos, Vec3.scale( pose_twist_dir, 0.8 ).add( pos ), "orange" );
    */

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ik.look_dir.copy( pose_look_dir );
    ik.twist_dir.copy( pose_twist_dir );
  }

  static spine( rig, chain, ik_pose, look_dir, twist_dir ){
    let idx_ary		= [ chain.first(), chain.last() ], // First and Last Bone of the Chain.
      q_inv		= new Quat(),
      v_look_dir	= new Vec3(),
      v_twist_dir	= new Vec3(),
      j			= 0,
      pose, bind;

    for( let i of idx_ary ){
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // First get reference to the Bones
      bind = rig.tpose.bones[ i ];
      pose = rig.pose.bones[ i ];

      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Create Quat Inverse Direction
      q_inv.from_invert( bind.world.rot );
      v_look_dir.from_quat( q_inv, look_dir );
      v_twist_dir.from_quat( q_inv, twist_dir );

      // Transform the Inv Dir by the Animated Pose to get their direction
      // Reusing Variables to save on memory / initialization
      v_look_dir.from_quat( pose.world.rot, v_look_dir );
      v_twist_dir.from_quat( pose.world.rot, v_twist_dir );

      /* DEBUG
      let pos = pose.world.pos;
      App.Debug.pnt( pos, "orange", 0.03, 6 );
      App.Debug.ln( pos, Vec3.scale( v_look_dir, 0.12 ).add(pos), "yellow" );
      App.Debug.ln( pos, Vec3.scale( v_twist_dir, 0.12 ).add(pos), "yellow" );
      */

      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Save IK
      ik_pose.spine[ j ].look_dir.copy( v_look_dir );
      ik_pose.spine[ j ].twist_dir.copy( v_twist_dir );
      j++;
    }

  }

}
