import {Vec3,Quat,Transform }	from "./fungi/maths/Maths.js";
import IKTarget 		from "./fungi.armature/IKTarget.js";

// Hold the IK Information, then apply it to a Rig
export class IKPose{
  target = new IKTarget();		// IK Solvers

  hip = {
    bind_height	: 0,			// Use to help Scale movement.
    movement	: new Vec3(),	// How Much Movement the Hip did in world space
    dir			: new Vec3(),
    twist		: 0,
  };

  foot_l = { look_dir:new Vec3(), twist_dir:new Vec3() };
  foot_r = { look_dir:new Vec3(), twist_dir:new Vec3() };

  // IK Data for limbs is first the Direction toward the End Effector,
  // The scaled length to the end effector, plus the direction that
  // the KNEE or ELBOW is pointing. For IK Targeting, Dir is FORWARD and
  // joint dir is UP
  leg_l = { len_scale:0,	dir:new Vec3(),	joint_dir:new Vec3() };
  leg_r = { len_scale:0,	dir:new Vec3(), joint_dir:new Vec3() };
  arm_l = { len_scale:0,	dir:new Vec3(),	joint_dir:new Vec3() };
  arm_r = { len_scale:0,	dir:new Vec3(), joint_dir:new Vec3() };

  spine = [
    { look_dir: new Vec3(), twist_dir: new Vec3() },
    { look_dir: new Vec3(), twist_dir: new Vec3() },
  ];

  head = { look_dir: new Vec3(), twist_dir: new Vec3() };

  /**
   *
   * @param {IKRig} rig
   */
  apply_rig( rig ){
    this.apply_hip( rig );

    this.apply_limb( rig, rig.chains.leg_l, this.leg_l );
    this.apply_limb( rig, rig.chains.leg_r, this.leg_r );

    this.apply_look_twist( rig, rig.points.foot_l, this.foot_l, Vec3.FORWARD, Vec3.UP );
    this.apply_look_twist( rig, rig.points.foot_r, this.foot_r, Vec3.FORWARD, Vec3.UP );

    this.apply_spline( rig, rig.chains.spine, this.spine, Vec3.UP, Vec3.FORWARD );

    if( rig.chains.arm_l ) this.apply_limb( rig, rig.chains.arm_l, this.arm_l );
    if( rig.chains.arm_r ) this.apply_limb( rig, rig.chains.arm_r, this.arm_r );

    this.apply_look_twist( rig, rig.points.head, this.head, Vec3.FORWARD, Vec3.UP );
  }

  /**
   *
   * @param {IKRig} rig
   */
  apply_hip( rig ){
    // First step is we need to get access to the Rig's TPose and Pose Hip Bone.
    // The idea is to transform our Bind Pose into a New Pose based on IK Data
    let b_info	= rig.points.hip,
      bind 	= rig.tpose.bones[ b_info.idx ],	// TPose which is our Bind Pose
      pose 	= rig.pose.bones[ b_info.idx ];		// Our Working Pose.

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Apply IK Swing & Twist ( HANDLE ROTATION )
    // When we compute the IK Hip, We used quaternion invert direction and defined that
    // the hip always points in the FORWARD Axis, so We can use that to quicky get Swing Rotation
    // Take note that vegeta and roborex's Hips are completely different but by using that inverse
    // direction trick, we are easily able to apply the same movement to both.
    let p_rot	= rig.pose.get_parent_rot( b_info.idx );	// Incase the Hip isn't the Root bone, but in our example they are.
    let b_rot	= Quat.mul( p_rot, bind.local.rot );		// Add LS rot of the hip to the WS Parent to get its WS Rot.
    let q		= Quat
      .unit_vecs( Vec3.FORWARD, this.hip.dir )	// Create Swing Rotation
      .mul( b_rot );								// Apply it to our WS Rotation

    // If There is a Twist Value, Apply that as a PreMultiplication.
    if( this.hip.twist != 0 ) q.pmul_axis_angle( this.hip.dir, this.hip.twist );

    // In the end, we need to convert to local space. Simply premul by the inverse of the parent
    q.pmul_invert( p_rot );

    rig.pose.set_bone( b_info.idx, q ); // Save LS rotation to pose

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TRANSLATION
    let h_scl	= bind.world.pos.y / this.hip.bind_height;	// Create Scale value from Src's Hip Height and Target's Hip Height
    let pos		= Vec3
      .scale( this.hip.movement, h_scl )		// Scale the Translation Differnce to Match this Models Scale
      .add( bind.world.pos );					// Then Add that change to the TPose Position

    // MAYBE we want to keep the stride distance exact, we can reset the XZ positions
    // BUT we need to keep the Y Movement scaled, else our leg IK won't work well since
    // our source is taller then our targets, this will cause our target legs to always
    // straighten out.
    //pos.x = this.hip.movement.x;
    //pos.z = this.hip.movement.z;

    rig.pose.set_bone( b_info.idx, null, pos );	// Save Position to Pose
  }

  apply_limb( rig, chain, limb, grounding=0 ){
    // The IK Solvers I put together takes transforms as input, not rotations.
    // The first thing we need is the WS Transform of the start of the chain
    // plus the parent's WS Transform. When are are building a full body IK
    // We need to do things in a certain order to build things correctly.
    // So before we can do legs, we need the hip/root to be moved to where it needs to go
    // The issue is that when people walk, you are actually falling forward, you catch
    // yourself when your front foot touches the floor, in the process you lift yourself
    // up a bit. During a whole walk, or run cycle, a person's hip is always moving up and down
    // Because of that, the distance from the Hip to the floor is constantly changing
    // which is important if we want to have the legs stretch correctly since each IK leg
    // length scale is based on the hip being at a certain height at the time.
    let p_tran = new Transform(),
      c_tran = new Transform();

    rig.pose.get_parent_world( chain.first(), p_tran, c_tran );

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // How much of the Chain length to use to calc End Effector
    let len = ( rig.leg_len_lmt || chain.len ) * limb.len_scale;

    // Next we pass our into to the Target which does a some pre computations that solvers may need.
    this.target.from_pos_dir( c_tran.pos, limb.dir, limb.joint_dir, len );	// Setup IK Target

    if( grounding ) this.apply_grounding( grounding );

    // Each Chain is assigned a IK Solver that will bend the bones to the right places
    let solver = chain.ik_solver || "limb";

    // The IK Solver will update the pose with the results of the operation. We pass in the
    // parent for it to use it to return things back into local space.
    this.target[ solver ]( chain, rig.tpose, rig.pose, p_tran );
  }

  apply_look_twist( rig, b_info, ik, look_dir, twist_dir ){
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // First we need to get the WS Rotation of the parent to the Foot
    // Then Add the Foot's LS Bind rotation. The idea is to see where
    // the foot will currently be if it has yet to have any rotation
    // applied to it.
    let bind 	= rig.tpose.bones[ b_info.idx ],
      pose 	= rig.pose.bones[ b_info.idx ];

    let p_rot 	= rig.pose.get_parent_rot( b_info.idx );
    let c_rot 	= Quat.mul( p_rot, bind.local.rot );

    /* DEBUG
    let p_tran	= rig.pose.get_parent_world( b_info.idx );
    let c_tran	= Transform.add( p_tran, bind.local );
    let tpos	= Vec3.add( c_tran.pos, [1,0,0] );		// Model was shifted, Add that Shift to keep things aligned.
    App.Debug.pnt( tpos, "yellow", 0.05, 1 );			// See Where the Foot is in 3D Space
    */

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Next we need to get the Foot's Quaternion Inverse Direction
    // Which matches up with the same Directions used to calculate the IK
    // information.
    let q_inv 			= Quat.invert( bind.world.rot ),
      alt_look_dir	= Vec3.transform_quat( look_dir, q_inv ),
      alt_twist_dir	= Vec3.transform_quat( twist_dir, q_inv );

    // After the HIP was moved and The Limb IK is complete, This is where
    // the ALT Look Direction currently points to.
    let now_look_dir = Vec3.transform_quat( alt_look_dir, c_rot );

    /* DEBUG - Here we can see where the bone is currently pointing to and where we need it to point
    App.Debug.ln( tpos, Vec3.scale( now_look_dir, 0.5 ).add( tpos ), "yellow" );	// Current Direction
    App.Debug.ln( tpos, Vec3.scale( ik.look_dir, 0.4 ).add( tpos ), "white" );		// Our Target IK Direction
    */

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Now we start building out final rotation that we
    // want to apply to the bone to get it pointing at the
    // right direction and twisted to match the original animation.
    let rot = Quat
      .unit_vecs( now_look_dir, ik.look_dir )	// Create our Swing Rotation
      .mul( c_rot );							// Then Apply to our foot

    /* DEBUG SWING ROTATION
    let new_look_dir	= Vec3.transform_quat( alt_look_dir, rot );
    let new_twist_dir	= Vec3.transform_quat( alt_twist_dir, rot );
    App.Debug.ln( tpos, Vec3.scale( new_look_dir, 0.8 ).add( tpos ), "yellow" );	// Current Directions
    App.Debug.ln( tpos, Vec3.scale( new_twist_dir, 0.8 ).add( tpos ), "yellow" );
    App.Debug.ln( tpos, Vec3.scale( ik.look_dir, 0.6 ).add( tpos ), "white" );		// Target Directions
    App.Debug.ln( tpos, Vec3.scale( ik.twist_dir, 0.6 ).add( tpos ), "white" );
    */

    // Now we need to know where the Twist Direction points to after
    // swing rotation has been applied. Then use it to compute our twist rotation.
    let now_twist_dir	= Vec3.transform_quat( alt_twist_dir, rot );
    let twist 			= Quat.unit_vecs( now_twist_dir, ik.twist_dir  );
    rot.pmul( twist );	// Apply Twist

    /* DEBUG SWING ROTATION
    let new_look_dir	= Vec3.transform_quat( alt_look_dir, rot );
    let new_twist_dir	= Vec3.transform_quat( alt_twist_dir, rot );
    App.Debug.ln( tpos, Vec3.scale( new_look_dir, 0.8 ).add( tpos ), "yellow" );	// Current Directions
    App.Debug.ln( tpos, Vec3.scale( new_twist_dir, 0.8 ).add( tpos ), "yellow" );
    App.Debug.ln( tpos, Vec3.scale( ik.look_dir, 0.6 ).add( tpos ), "white" );		// Target Directions
    App.Debug.ln( tpos, Vec3.scale( ik.twist_dir, 0.6 ).add( tpos ), "white" );
    */

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    rot.pmul_invert( p_rot );				// To Local Space
    rig.pose.set_bone( b_info.idx, rot );	// Save to pose.
  }

  apply_grounding( y_lmt ){
    // Once we have out IK Target setup, We can use its data to test various things
    // First we can test if the end effector is below the height limit. Each foot
    // may need a different off the ground offset since the bones rarely touch the floor
    // perfectly.
    if( this.target.end_pos.y >= y_lmt ) return;

    /* DEBUG IK TARGET */
    let tar		= this.target,
      posA	= Vec3.add( tar.start_pos, [-1,0,0] ),
      posB	= Vec3.add( tar.end_pos, [-1,0,0] );
    App.Debug
      .pnt( posA , "yellow", 0.05, 6 )
      .pnt( posB , "white", 0.05, 6 )
      .ln( posA, posB , "yellow", "white", true );

    // Where on the line between the Start and end Points would work for our
    // Y Limit. An easy solution is to find the SCALE based on doing a 1D Scale
    //operation on the Y Values only. Whatever scale value we get with Y we can use on X and Z

    let a = this.target.start_pos,
      b = this.target.end_pos,
      s = (y_lmt - a.y) / (b.y - a.y); // Normalize Limit Value in the Max/Min Range of Y.

    // Change the end effector of our target
    this.target.end_pos.set(
      (b.x-a.x) * s + a.x, // We scale the 1D Range then apply it to the start
      y_lmt,
      (b.z-a.z) * s + a.z
    );

    /* DEBUG NEW END EFFECTOR */
    App.Debug.pnt( Vec3.add(this.target.end_pos,[-1,0,0]) , "orange", 0.05, 6 );

    // Since we changed the end effector, lets update the Sqr Length and Length of our target
    // This is normally computed by our IK Target when we set it, but since I didn't bother
    // to create a method to update the end effector, we need to do these extra updates.
    // IDEALLY i should make that function to prevent bugs :)
    this.target.len_sqr		= Vec3.len_sqr( this.target.start_pos, this.target.end_pos );
    this.target.len			= Math.sqrt( this.target.len_sqr );
  }

  apply_spline( rig, chain, ik, look_dir, twist_dir ){
    // For the spline, we have the start and end IK directions. Since spines can have various
    // amount of bones, the simplest solution is to lerp from start to finish. The first
    // spine bone is important to control offsets from the hips, and the final one usually
    // controls the chest which dictates where the arms and head are going to be located.
    // Anything between is how the spine would kind of react.

    // Since we are building up the Skeleton, We look at the pose object to know where the Hips
    // currently exist in World Space.

    let cnt			= chain.cnt - 1,								// How Many Spine Bones to deal with
      p_tran		= rig.pose.get_parent_world( chain.first() ),	// World Space Transform of the spine's parent, usually the hips
      c_tran		= new Transform(),
      ik_look		= new Vec3(),
      ik_twist	= new Vec3(),
      alt_look	= new Vec3(),
      alt_twist	= new Vec3(),
      now_look	= new Vec3(),
      now_twist	= new Vec3(),
      q 			= new Quat(),
      rot 		= new Quat();

    let t, idx, bind, pose;

    for( let i=0; i <= cnt; i++ ){
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Prepare for the Iteration
      idx		= chain.bones[ i ].idx;		// Bone Index
      bind	= rig.tpose.bones[ idx ];	// Bind Values of the Bone
      t 		= (i / cnt);// ** 2;		// The Lerp Time, be 0 on first bone, 1 at final bone, Can use curves to distribute the lerp differently

      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Lerp our Target IK Directions for this bone
      ik_look.from_lerp(	ik[0].look_dir,		ik[1].look_dir,		t );
      ik_twist.from_lerp(	ik[0].twist_dir,	ik[1].twist_dir,	t );

      // Compute our Quat Inverse Direction, using the Defined Look&Twist Direction
      q.from_invert( bind.world.rot );
      alt_look.from_quat( q, look_dir );
      alt_twist.from_quat( q, twist_dir );

      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      c_tran.from_add( p_tran, bind.local );		// Get bone in WS that has yet to have any rotation applied
      now_look.from_quat( c_tran.rot, alt_look );	// What direction is the bone point looking now, without any extra rotation

      /* DEBUG
      let pos = Vec3.add( c_tran.pos, [1,0,0] );
      App.Debug.pnt( pos, "yellow", 0.02, 1 );
      */

      rot
        .from_unit_vecs( now_look, ik_look )	// Create our Swing Rotation
        .mul( c_tran.rot );						// Then Apply to our Bone, so its now swong to match the swing direction.

      now_twist.from_quat( rot, alt_twist );		// Get our Current Twist Direction from Our Swing Rotation
      q.from_unit_vecs( now_twist, ik_twist  );	// Create our twist rotation
      rot.pmul( q );								// Apply Twist so now it matches our IK Twist direction

      /*
      now_twist.from_quat( rot, alt_twist );
      App.Debug.ln( pos, Vec3.scale( now_twist, 0.5 ).add(pos), "yellow" );
      App.Debug.ln( pos, Vec3.scale( ik_twist, 0.3 ).add(pos), "white" );
      //break;
      */
      rot.pmul_invert( p_tran.rot );				// To Local Space

      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      rig.pose.set_bone( idx, rot );						// Save back to pose
      if( t != 1 ) p_tran.add( rot, bind.local.pos, bind.local.scl );	// Compute the WS Transform for the next bone in the chain.
    }
  }
}
