use godot::classes::*;
use godot::prelude::*;

use super::wheel_joint_options::WheelJointOptions;
use super::rapier_joint_base::RapierJointBase;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::types::*;
use std::f32::consts::PI;

pub struct RapierWheelJoint2D {
    options: WheelJointOptions,
    base: RapierJointBase,
}

impl RapierWheelJoint2D {
    pub fn new(
        id: RapierId,
        rid: Rid,
        anchor_a: Vector,
        anchor_b: Vector,
        body_a: &RapierCollisionObject,
        body_b: &RapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
    ) -> Self {
        let invalid_joint = Self {
            options: WheelJointOptions::default(),
            base: RapierJointBase::default(),
        };
        let body_a_rid = body_a.get_base().get_rid();
        let body_b_rid = body_b.get_base().get_rid();
        if body_a_rid == body_b_rid {
            return invalid_joint;
        }
        if !body_a.get_base().is_valid()
            || !body_b.get_base().is_valid()
            || body_a.get_base().get_space_id() != body_b.get_base().get_space_id()
        {
            return invalid_joint;
        }
        let rapier_anchor_a = body_a.get_base().get_inv_transform() * anchor_a;
        let rapier_anchor_b = body_b.get_base().get_inv_transform() * anchor_b;
        let rest_length = (anchor_a - anchor_b).length();
        let space_handle = body_a.get_base().get_space_id();
        let space_id = body_a.get_base().get_space_id();
        let handle = physics_engine.joint_create_wheel(
            space_handle,
            body_a.get_base().get_body_handle(),
            body_b.get_base().get_body_handle(),
            vector_to_rapier(rapier_anchor_a),
            vector_to_rapier(rapier_anchor_b),
            false,
            false,
            true,
        );
        Self {
            options: WheelJointOptions::default(),
            base: RapierJointBase::new(id, rid, space_id, space_handle, handle),
        }
    }

    pub fn set_param(
        &mut self,
        param: physics_server_2d::WheelJointParam,
        value: f32,
        physics_engine: &mut PhysicsEngine,
    ) {

        match param {
            physics_server_2d::WheelJointParam::X_STIFFNESS => {
                self.options.x_stiffness = value;
            }
            physics_server_2d::WheelJointParam::X_DAMPING => {
                self.options.x_damping = value;
            }
            physics_server_2d::WheelJointParam::X_LOWER_LIMIT => {
                self.options.x_lower_limit = value;
            }
            physics_server_2d::WheelJointParam::X_UPPER_LIMIT => {
                self.options.x_upper_limit = value;
            }
            physics_server_2d::WheelJointParam::Y_STIFFNESS => {
                self.options.y_stiffness = value;
            }
            physics_server_2d::WheelJointParam::Y_DAMPING => {
                self.options.y_damping = value;
            }
            physics_server_2d::WheelJointParam::Y_LOWER_LIMIT => {
                self.options.y_lower_limit = value;
            }
            physics_server_2d::WheelJointParam::Y_UPPER_LIMIT => {
                self.options.y_upper_limit = value;
            }
            _ => {}
        }

        if !self.base.is_valid() {
            return;
        }

        physics_engine.joint_change_wheel_params(
            self.base.get_space_id(),
            self.base.get_handle(),
            &self.options,
        );
    }

    pub fn get_param(
        &self,
        param: physics_server_2d::WheelJointParam
    ) -> f32 {
        match param {
            physics_server_2d::WheelJointParam::X_STIFFNESS => {
                return self.options.x_stiffness;
            }
            physics_server_2d::WheelJointParam::X_DAMPING => {
                return self.options.x_damping;
            }
            physics_server_2d::WheelJointParam::X_LOWER_LIMIT => {
                return self.options.x_lower_limit;
            }
            physics_server_2d::WheelJointParam::X_UPPER_LIMIT => {
                return self.options.x_upper_limit;
            }
            physics_server_2d::WheelJointParam::Y_STIFFNESS => {
                return self.options.y_stiffness;
            }
            physics_server_2d::WheelJointParam::Y_DAMPING => {
                return self.options.y_damping;
            }
            physics_server_2d::WheelJointParam::Y_LOWER_LIMIT => {
                return self.options.y_lower_limit;
            }
            physics_server_2d::WheelJointParam::Y_UPPER_LIMIT => {
                return self.options.y_upper_limit;
            }
            _ => 0.0,
        }
    }

    pub fn set_flag(
        &mut self,
        flag: physics_server_2d::WheelJointFlag,
        enabled: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
        match flag {
            physics_server_2d::WheelJointFlag::X_LIMITS_ENABLED => {
                self.options.x_limits_enabled = enabled;
            }
            physics_server_2d::WheelJointFlag::Y_LIMITS_ENABLED => {
                self.options.y_limits_enabled = enabled;
            }
            _ => { },
        }

        if !self.base.is_valid() {
            return;
        }

        physics_engine.joint_change_wheel_params(
            self.base.get_space_id(),
            self.base.get_handle(),
            &self.options,
        );
    }

    pub fn get_flag(
        &self,
        flag: physics_server_2d::WheelJointFlag,
    ) -> bool {

        match flag {
            physics_server_2d::WheelJointFlag::X_LIMITS_ENABLED => {
                return self.options.x_limits_enabled;
            }
            physics_server_2d::WheelJointFlag::Y_LIMITS_ENABLED => {
                return self.options.y_limits_enabled;
            }
            _ => false,
        }
    }

}

impl IRapierJoint for RapierWheelJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::WHEEL
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }

    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }
}
