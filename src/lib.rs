// Disable std if no_std crate
#![cfg_attr(feature = "no_std", no_std)]

// Definition of the Motor Position
#[derive(Clone, Copy, Debug)]
pub enum MotorPosition {
    FrontLeft,
    FrontRight,
    BackLeft,
    BackRight,
    Left,
    Right,
    Front,
    Back,
    All,
}

#[derive(Debug, Default)]
pub struct MotorState {
    pub front_left: f32,
    pub front_right: f32,
    pub rear_left: f32,
    pub rear_right: f32,
    pub armed: bool,
}

impl core::convert::Into<u8> for MotorPosition {
    fn into(self) -> u8 {
        use MotorPosition::*;
        match self {
            FrontLeft => 0x01,
            FrontRight => 0x02,
            BackLeft => 0x03,
            BackRight => 0x04,
            Left => 0xF1,
            Right => 0xF2,
            Front => 0xF3,
            Back => 0xF4,
            All => 0xFF,
        }
    }
}

impl core::convert::TryFrom<u8> for MotorPosition {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use MotorPosition::*;
        match value {
            0x01 => Ok(FrontLeft),
            0x02 => Ok(FrontRight),
            0x03 => Ok(BackLeft),
            0x04 => Ok(BackRight),
            0xF1 => Ok(Left),
            0xF2 => Ok(Right),
            0xF3 => Ok(Front),
            0xF4 => Ok(Back),
            0xFF => Ok(All),
            _ => Err(()),
        }
    }
}

/// List of possible Commands for serial communication
#[derive(Clone, Copy, Debug)]
pub enum Command {
    StartMotor,
    StopMotor,
    GetMotionState,
    SendMotionState(nalgebra::Vector3<f32>, bool),
    ToggleLed,
}

impl Command {
    /// Convert to a byte array representation. Returns Err if for some reason the byte could not
    /// be pushed to the vector
    #[cfg(feature = "no_std")]
    fn to_vec(self, out: &mut heapless::Vec<u8, heapless::consts::U32>) -> Result<(), ()> {
        // Clear the Vecotr
        out.clear();

        // Add the Cmd Identifyer
        out.push(self.into()).map_err(|_| ())?;

        use Command::*;
        match self {
            ToggleLed | GetMotionState => (), // No additional Info
            StartMotor | StopMotor => (),
            SendMotionState(angle_vel, armed) => {
                angle_vel.iter().try_for_each(|vel| {
                    vel.to_bits()
                        .to_be_bytes()
                        .iter()
                        .try_for_each(|byte| out.push(*byte).map_err(|_| ()))
                })?;
                out.push(if armed { 1 } else { 0 }).map_err(|_| ())?;
            }
        };

        Ok(())
    }

    /// Convert to a byte array representation.
    #[cfg(not(feature = "no_std"))]
    fn to_vec(self) -> Vec<u8> {
        // Clear the Vecotr
        let mut out = Vec::new();

        // Add the Cmd Identifyer
        out.push(self.into());

        use Command::*;
        match self {
            ToggleLed | GetMotionState => (), // No additional Info
            StartMotor | StopMotor => (),
            SendMotionState(angle_vel, armed) => {
                angle_vel.iter().for_each(|vel| {
                    vel.to_bits()
                        .to_be_bytes()
                        .iter()
                        .for_each(|byte| out.push(*byte))
                });
                out.push(if armed { 1 } else { 0 });
            }
        };
        out
    }

    /// Convert a given array to a Command. Returns Err if length is not correct or an error occured
    /// while parsing.
    fn from_byte_array(data: &[u8]) -> Result<Command, ()> {
        let mut iter = data.iter();

        use core::convert::TryFrom;
        use Command::*;

        // Decode the command
        let cmd = iter
            .next()
            .ok_or(())
            .and_then(|val| Command::try_from(*val))?;

        // Try to Decode the payload
        match cmd {
            ToggleLed => Ok(ToggleLed),           // No additional Info
            GetMotionState => Ok(GetMotionState), // No additional Info
            StartMotor => Ok(StartMotor),
            StopMotor => Ok(StopMotor),
            SendMotionState(_, _) => {
                let mut buffer = [0_u8; 4];
                let mut angle_vel = nalgebra::Vector3::<f32>::zeros();
                for el in angle_vel.iter_mut() {
                    // Try to get the next 4 bytes
                    buffer.iter_mut().try_for_each(|byte| {
                        *byte = *iter.next().ok_or(())?;
                        Ok(())
                    })?;
                    use core::convert::TryInto;
                    // Try to convert to a float
                    *el = f32::from_bits(u32::from_be_bytes(buffer.try_into().map_err(|_| ())?));
                }
                // Try to get armed state
                let armed = iter.next().ok_or(()).and_then(|&val| {
                    if val == 0 {
                        Ok(false)
                    } else if val == 1 {
                        Ok(true)
                    } else {
                        Err(())
                    }
                })?;
                Ok(SendMotionState(angle_vel, armed))
            }
        }
    }

    /// Convert from a slip coded slice of bytes to a Command
    #[cfg(feature = "no_std")]
    pub fn from_slip(input: &heapless::Vec<u8, heapless::consts::U32>) -> Result<Self, ()> {
        let mut decoded_bytes = heapless::Vec::<u8, heapless::consts::U32>::new();

        // Decode Slip
        rc_framing::framing::decode(input, &mut decoded_bytes)?;

        // Convert to Command
        Command::from_byte_array(&decoded_bytes)
    }

    /// Convert from a slip coded vector of bytes to a Command
    #[cfg(not(feature = "no_std"))]
    pub fn from_slip(input: &Vec<u8>) -> Result<Self, ()> {
        // Decode Slip
        let decoded_bytes = rc_framing::framing::decode(input)?;

        // Convert to Command
        Command::from_byte_array(&decoded_bytes)
    }

    /// Convert a Command to bytes. Fails if the target Vector is not long enough.
    #[cfg(feature = "no_std")]
    pub fn to_slip(
        self,
        output: &mut heapless::Vec<u8, heapless::consts::U32>,
    ) -> Result<usize, ()> {
        let mut bytes = heapless::Vec::<u8, heapless::consts::U32>::new();

        // Convert to Array
        self.to_vec(&mut bytes)?;

        // Encode with SLIP
        rc_framing::framing::encode(&bytes, output)?;

        Ok(output.len())
    }

    /// Convert a Command to bytes. Fails if the target Vector is not long enough.
    #[cfg(not(feature = "no_std"))]
    pub fn to_slip(self) -> Vec<u8> {
        // Convert to Array
        let bytes = self.to_vec();

        // Encode with SLIP
        rc_framing::framing::encode(&bytes)
    }
}

impl core::convert::TryFrom<u8> for Command {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use Command::*;
        match value {
            1 => Ok(ToggleLed),
            10 => Ok(StartMotor),
            11 => Ok(StopMotor),
            20 => Ok(GetMotionState),
            21 => Ok(SendMotionState(nalgebra::Vector3::<f32>::zeros(), false)),
            _ => Err(()),
        }
    }
}

impl core::convert::Into<u8> for Command {
    fn into(self) -> u8 {
        use Command::*;
        match self {
            ToggleLed => 1,
            StartMotor => 10,
            StopMotor => 11,
            GetMotionState => 20,
            SendMotionState(_, _) => 21,
        }
    }
}
