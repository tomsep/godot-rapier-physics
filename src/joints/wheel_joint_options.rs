pub(crate) struct WheelJointOptions {
    pub x_stiffness: f32,
    pub x_damping: f32,
    pub x_lower_limit: f32,
    pub x_upper_limit: f32,

    pub y_stiffness: f32,
    pub y_damping: f32,
    pub y_lower_limit: f32,
    pub y_upper_limit: f32,

    pub  x_limits_enabled: bool,
    pub  y_limits_enabled: bool,
}

impl Default for WheelJointOptions {
    fn default() -> Self {
        WheelJointOptions {
            x_stiffness: 20.0,
            x_damping: 1.0,
            x_lower_limit: 0.0,
            x_upper_limit: 0.0,

            y_stiffness: 20.0,
            y_damping: 1.0,
            y_lower_limit: 0.0,
            y_upper_limit: 0.0,

            x_limits_enabled: false,
            y_limits_enabled: false,
        }
    }
}